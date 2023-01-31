// Copyright 2023 ATL Robotics, USA
#include "teensy_interface/teensy_interface_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "udp_server/udp_server.hpp"
#include "parameter/parameter.hpp"
#include "msg_utils/msg_utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <exception>
#include <cerrno>
#include <utility>

using std::string;

constexpr static std::size_t npl = 6;
constexpr static std::size_t npb = 8;
constexpr static std::size_t nhml = 4;
constexpr static std::size_t nEngMsg = 4;
constexpr static std::size_t nBuzz = 1;
constexpr static std::size_t nTrimTabs = 0;
constexpr static std::size_t nFuelTanks = 2;

namespace atl
{

TeensyInterfaceComponent::TeensyInterfaceComponent(const rclcpp::NodeOptions & options)
: Node("teensy_interface", options),
  udp_(std::make_unique<atl::UDPServer>())
{
  initParamsInNode(*this, prm_, "", false, true);

  buzzMsg_.resize(nBuzz, std::nullopt);
  enginePowerData_.resize(prm_.n_engines);

  if (prm_.n_engines > nEngMsg || prm_.n_engines < 1) {
    throw std::runtime_error("Unsupported number of engines");
  }

  userTrimInput_.resize(prm_.n_engines);
  desTrimInput_.resize(prm_.n_engines);
  trimIntegrators_.resize(prm_.n_engines);

  // Set parameters on trim integrators
  for (auto & trim_integrator : trimIntegrators_) {
    trim_integrator.set_params({true, prm_.trim.sat_min, prm_.trim.sat_max});
    trim_integrator.set_clock(get_clock());
  }

  // Create the subscriptions
  rclcpp::SensorDataQoS inputQoS;
  inputQoS.keep_last(1);
  subInput_ = create_subscription<atl_msgs::msg::EnginesInput>(
    "input", inputQoS,
    [this](atl_msgs::msg::EnginesInput::SharedPtr msg) {
      subInputCb(std::move(msg));
    });

  subBuzz_ = create_subscription<atl_msgs::msg::Buzzers>(
    "buzzers", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::Buzzers::SharedPtr msg) {
      subBuzzCb(std::move(msg));
    });
  subPanelLights_ = create_subscription<atl_msgs::msg::Lights>(
    "lights_panel", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::Lights::SharedPtr msg) {
      subPanelLightsCb(std::move(msg));
    });
  subHMLights_ = create_subscription<atl_msgs::msg::Lights>(
    "lights_helm_master", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::Lights::SharedPtr msg) {
      subHMLightsCb(std::move(msg));
    });
  subEngineTrim_ = create_subscription<atl_msgs::msg::TrimsInput>(
    "engines_trim", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::TrimsInput::SharedPtr msg) {
      subEngineTrimCb(std::move(msg));
    });
  subTrimTabs_ = create_subscription<atl_msgs::msg::TrimsInput>(
    "trim_tabs", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::TrimsInput::SharedPtr msg) {
      subTrimTabsCb(std::move(msg));
    });
  subEnginePower_ = create_subscription<atl_msgs::msg::EnginesPowerInput>(
    "engines_power", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::EnginesPowerInput::SharedPtr msg) {
      subEnginePowerCb(std::move(msg));
    });

    // --- 
  subCtrlStatus_ = create_subscription<atl_msgs::msg::CtrlStatus>(
    "/systems/control/ctrl_status", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::CtrlStatus::SharedPtr msg) {
      subCtrlStatusCb(std::move(msg));
    });
  subCortexStatus_ = create_subscription<atl_msgs::msg::CortexStatus>(
    "cortex_status", rclcpp::SystemDefaultsQoS(),
    [this](atl_msgs::msg::CortexStatus::SharedPtr msg) {
      subCortexStatusCB(std::move(msg));
    });

  // Create the boat state publisher
  pubStatus_ = create_publisher<atl_msgs::msg::BcuStatus>(
    "bcu_status", rclcpp::SystemDefaultsQoS());

  // Create the engine state publisher
  pubEngine_ = create_publisher<atl_msgs::msg::EnginesState>(
    "engine_state", rclcpp::SensorDataQoS());

  // Create the helm master publisher
  pubHM_ = create_publisher<sensor_msgs::msg::Joy>(
    "helm_master", rclcpp::SystemDefaultsQoS());

  // Create the panel buttons publisher
  pubPanel_ = create_publisher<sensor_msgs::msg::Joy>(
    "panel", rclcpp::SystemDefaultsQoS());

  // Create the steering publisher
  pubSteer_ = create_publisher<sensor_msgs::msg::Joy>(
    "steering", rclcpp::SystemDefaultsQoS());

  // Create the fuel publisher
  pubFuel_ = create_publisher<atl_msgs::msg::FuelTanks>(
    "fuel", rclcpp::SystemDefaultsQoS());

  // set up UDP communication
  udp_->init(
    prm_.udp.receive_buffer_size,
    prm_.udp.send_port,
    prm_.udp.teensy_ip,
    prm_.udp.receive_port,
    prm_.udp.teensy_ip
  );

  udp_->subscribe(
    [this](const UDPServer::UDPMsg & msg) {
      udpCb(msg);
    });

  RCLCPP_INFO(get_logger(), "Teensy Interface Node started");
  rclcppLogParams(get_logger(), prm_, 2, '_');
}

void TeensyInterfaceComponent::udpCb(const UDPServer::UDPMsg & msg)
{
  constexpr std::size_t msgLen = (nEngMsg * 23 + 18 + npb + nFuelTanks) * 4;
  if (msg.data.size() != msgLen) {
    RCLCPP_ERROR(
      get_logger(), "Teensy UDP message must be of length %lu, received %lu.",
      msgLen, msg.data.size());
    throw std::runtime_error("Teensy UDP message has incorrect size.");
  }

  const auto tNow = now();

  atl_msgs::msg::BcuStatus statusMsg;
  statusMsg.header.stamp = tNow;

  atl_msgs::msg::EnginesState enginesMsg;
  enginesMsg.header.stamp = tNow;
  enginesMsg.states.reserve(prm_.n_engines);
  enginesMsg.infos.reserve(prm_.n_engines);

  sensor_msgs::msg::Joy steerMsg;
  steerMsg.header.stamp = tNow;
  steerMsg.axes.resize(3);
  steerMsg.buttons.resize(4);

  sensor_msgs::msg::Joy helmMasterMsg;
  helmMasterMsg.header.stamp = tNow;
  helmMasterMsg.axes.resize(3);
  helmMasterMsg.buttons.resize(6);

  sensor_msgs::msg::Joy panelMsg;
  panelMsg.header.stamp = tNow;
  panelMsg.buttons.resize(npb);


  atl_msgs::msg::FuelTanks fuelMsg;
  fuelMsg.header.stamp = tNow;
  fuelMsg.levels.resize(nFuelTanks);

  std::size_t oft = 0;

  // BCU status
  statusMsg.status = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  pubStatus_->publish(std::move(statusMsg));

  // Steering axis, throttle lever axes, and trim buttons
  steerMsg.axes[0] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.axes[1] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.axes[2] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.buttons[0] = (*(reinterpret_cast<const int32_t *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.buttons[1] = (*(reinterpret_cast<const int32_t *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.buttons[2] = (*(reinterpret_cast<const int32_t *>(msg.data.data() + oft)));
  oft += 4;
  steerMsg.buttons[3] = (*(reinterpret_cast<const int32_t *>(msg.data.data() + oft)));
  oft += 4;

  // save user trim button inputs to global variable
  if (steerMsg.buttons[3] != 0) {
    std::fill(userTrimInput_.begin(), userTrimInput_.end(), steerMsg.buttons[3]);
  } else {
    userTrimInput_.front() = steerMsg.buttons[0];
    userTrimInput_.back() = steerMsg.buttons[2];
  }

  pubSteer_->publish(std::move(steerMsg));

  // Fuel
  for (std::size_t i = 0; i < nFuelTanks; i++) {
    fuelMsg.levels[i] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
    oft += 4;
  }
  pubFuel_->publish(std::move(fuelMsg));

  // Helm master axes and buttons
  helmMasterMsg.axes[0] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  helmMasterMsg.axes[1] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  helmMasterMsg.axes[2] = (*(reinterpret_cast<const float *>(msg.data.data() + oft)));
  oft += 4;
  helmMasterMsg.buttons[0] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  helmMasterMsg.buttons[1] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  helmMasterMsg.buttons[2] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  helmMasterMsg.buttons[3] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  helmMasterMsg.buttons[4] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  helmMasterMsg.buttons[5] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  pubHM_->publish(std::move(helmMasterMsg));

  // Panel buttons
  for (std::size_t i = 0; i < npb; i++) {
    panelMsg.buttons[i] = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
  }
  pubPanel_->publish(std::move(panelMsg));

  // Number of engines
  const std::size_t nEng = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
  oft += 4;
  if (nEng != prm_.n_engines) {
    RCLCPP_ERROR(
      get_logger(), "Teensy UDP message must contain %lu engines, received %lu.",
      prm_.n_engines, nEng);
    throw std::runtime_error("Teensy UDP message has incorrect number of engines.");
  }

  // Engine states
  for (std::size_t i = 0; i < prm_.n_engines; i++) {
    atl_msgs::msg::EngineState engineMsg;
    engineMsg.rpm = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineMsg.delta = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineMsg.clu = *(reinterpret_cast<const int32_t *>(msg.data.data() + oft));
    oft += 4;
    enginesMsg.states.push_back(std::move(engineMsg));

    atl_msgs::msg::EngineInfo engineInfo;
    engineInfo.flag = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.type = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.trim = *(reinterpret_cast<const float *>(msg.data.data() + oft));

    // Estimate trim
    const int trueTrimInput = userTrimInput_[i] != 0 ? userTrimInput_[i] : desTrimInput_[i];
    if (engineInfo.trim >= prm_.trim.sensor_threshold) {
      if (!trimIntegratorsInit_) {
        trimIntegrators_[i].reset(prm_.trim.sat_max, tNow);
      } else {
        if (trueTrimInput == 1) {
          if (trimIntegrators_[i].getValue() >= prm_.trim.trim_to_tilt_threshold) {
            trimIntegrators_[i].update(prm_.trim.v_tilt_up, tNow);
          } else {
            trimIntegrators_[i].update(prm_.trim.v_trim_up, tNow);
          }
        } else if (trueTrimInput == -1) {
          if (trimIntegrators_[i].getValue() >= prm_.trim.trim_to_tilt_threshold) {
            trimIntegrators_[i].update(prm_.trim.v_tilt_down, tNow);
          } else {
            trimIntegrators_[i].update(prm_.trim.v_trim_down, tNow);
          }
          if (trimIntegrators_[i].getValue() < prm_.trim.sensor_threshold) {
            trimIntegrators_[i].reset(prm_.trim.sensor_threshold, tNow);
          }
        } else {
          trimIntegrators_[i].update(0., tNow);
        }
      }
      engineInfo.trim = trimIntegrators_[i].getValue();
    } else {
      trimIntegrators_[i].reset(engineInfo.trim, tNow);
    }

    oft += 4;
    engineInfo.hours = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    // fuel_rate changes from l/hr to m^3/s
    engineInfo.fuel_rate = (*(reinterpret_cast<const float *>(msg.data.data() + oft))) / 3.6e6;
    oft += 4;
    engineInfo.starter_battery_voltage = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.aux_battery_voltage = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.oil_pressure = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.coolant_pressure = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.coolant_temp = *(reinterpret_cast<const float *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.check_eng_fault = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.eng_temp_fault = *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.low_oil_pressure_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.low_oil_level_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.low_fuel_pressure_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.low_system_voltage_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.low_coolant_level_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.water_in_fuel_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.rev_limit_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    engineInfo.throttle_pos_sensor_fault =
      *(reinterpret_cast<const uint32_t *>(msg.data.data() + oft));
    oft += 4;
    enginesMsg.infos.push_back(std::move(engineInfo));
  }

  if (iter_ == 0) {
    RCLCPP_INFO(get_logger(), "Discovered %lu engines :", prm_.n_engines);
    for (std::size_t i = 0; i < prm_.n_engines; i++) {
      RCLCPP_INFO(
        get_logger(), "  Engine %lu type : %s",
        i,
        msgs::EngineTypeEnum(enginesMsg.infos[i].type).c_str());
    }
  }

  pubEngine_->publish(std::move(enginesMsg));

  iter_++;
  trimIntegratorsInit_ = true;
}

void TeensyInterfaceComponent::subInputCb(atl_msgs::msg::EnginesInput::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);

  if (msg->inputs.size() != prm_.n_engines) {
    RCLCPP_ERROR(
      get_logger(), "EnginesInput message must have %lu inputs, received %lu.",
      prm_.n_engines, msg->inputs.size());
    throw std::runtime_error("EnginesInput message has wrong number of inputs.");
  }

  // forward message through UDP
  std::vector<uint8_t> u((3 + npl + nhml + 5 * nEngMsg + 1 + 1) * 4);
  std::size_t oft = 0;

  // Timestamp
  const uint64_t time_ns = now().nanoseconds();
  const uint32_t time_ms = static_cast<uint32_t>((time_ns - t0_) / 1000000U);
  memcpy(u.data() + oft, &time_ms, 4);
  oft += 4;

  // Active
  const uint32_t active = msg->active;
  memcpy(u.data() + oft, &active, 4);
  oft += 4;

  // Buzzer
  uint32_t buzzer = 0;
  if (buzzMsg_[0]) {
    buzzer = buzzMsg_[0].value().first > 0;
    if (buzzMsg_[0].value().first == atl_msgs::msg::Buzzers::BEEP) {
      buzzMsg_[0].value().second--;
      if (buzzMsg_[0].value().second <= 0) {
        buzzMsg_[0].reset();
      }
    }
  }
  memcpy(u.data() + oft, &buzzer, 4);
  oft += 4;

  // Helm master lights
  std::vector<uint32_t> HMLights(nhml, atl_msgs::msg::Light::OFF);
  if (HMLightsMsg_) {
    for (std::size_t i = 0; i < nhml; i++) {
      HMLights[i] = HMLightsMsg_->data[i].mode;
    }
  }
  for (std::size_t i = 0; i < nhml; i++) {
    memcpy(u.data() + oft, &HMLights[i], 4);
    oft += 4;
  }

  // Panel lights
  std::vector<uint32_t> panelLights(npl, atl_msgs::msg::Light::OFF);
  if (panelLightsMsg_) {
    for (std::size_t i = 0; i < npl; i++) {
      panelLights[i] = panelLightsMsg_->data[i].mode;
    }
  }
  for (std::size_t i = 0; i < npl; i++) {
    memcpy(u.data() + oft, &panelLights[i], 4);
    oft += 4;
  }

  // Engine trim
  std::vector<int32_t> engineTrim(prm_.n_engines, 0);
  if (engineTrimMsg_) {
    engineTrim = engineTrimMsg_->inputs;
  }

  for (std::size_t i = 0; i < prm_.n_engines; i++) {
    const float thr = msg->inputs[i].thr;
    const float del = msg->inputs[i].delta;
    const int32_t clu = msg->inputs[i].clu;

    const auto clu_enum = msgs::int64ToClutch(msg->inputs[i].clu);
    if (clu_enum == msgs::CLUTCH::UNKOWN) {
      RCLCPP_ERROR(get_logger(), "Clutch value for engine %lu outside of [[-1,1]] range", i);
      throw std::runtime_error("Incorrect clutch value");
    }

    memcpy(u.data() + oft, &thr, 4);
    oft += 4;

    memcpy(u.data() + oft, &del, 4);
    oft += 4;

    memcpy(u.data() + oft, &clu, 4);
    oft += 4;

    memcpy(u.data() + oft, &engineTrim[i], 4);
    oft += 4;

    memcpy(u.data() + oft, &enginePowerData_[i], 4);
    oft += 4;
  }

  // Set remaining engines to 0
  for (std::size_t i = prm_.n_engines; i < nEngMsg; i++) {
    const float thr = 0.f;
    memcpy(u.data() + oft, &thr, 4);
    oft += 4;

    const float del = 0.f;
    memcpy(u.data() + oft, &del, 4);
    oft += 4;

    const int32_t clu = 0;
    memcpy(u.data() + oft, &clu, 4);
    oft += 4;

    const int32_t trim = 0;
    memcpy(u.data() + oft, &trim, 4);
    oft += 4;

    const int32_t power = 0;
    memcpy(u.data() + oft, &power, 4);
    oft += 4;
  }

  // Sync byte
  const uint32_t sync = std::exchange(sync_, false);
  memcpy(u.data() + oft, &sync, 4);
  oft += 4;

  // Mode Info
  uint32_t modeActive;
  if (task_ > 0){
    modeActive = task_;
  } else {
    modeActive = ctrlMode_;
  }
  memcpy(u.data() + oft, &modeActive, 4);
  oft += 4;

  udp_->sendMsg(u);
}

// callbacks 
void TeensyInterfaceComponent::subCtrlStatusCb(atl_msgs::msg::CtrlStatus::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  std::string ctrlMode = msg->mode.data;
  // RCLCPP_INFO(get_logger(), "teensy_interface: Mode Received... Mode: %s", ctrlMode.c_str());

  if (ctrlMode == "manual"){
    ctrlMode_ = 2;
  } else if (ctrlMode == "cyber_joystick"){
    ctrlMode_ = 3;
  } else if (ctrlMode == "cyber_swan"){
    ctrlMode_ = 4;
  } else if (ctrlMode == "hold"){
    ctrlMode_ = 5;
  } else if (ctrlMode == "helm_master"){
    ctrlMode_ = 6;
  } else {
    ctrlMode_ = 7;
  }
}

void TeensyInterfaceComponent::subCortexStatusCB(atl_msgs::msg::CortexStatus::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  std::string task = msg->control_task.data;
  //RCLCPP_INFO(get_logger(), "teensy_interface: Task Received... Task: %s", task.c_str());
  
  if (task == "autodocking"){
    task_ = 1;
  } else {
    task_ = 0;
  }
}

void TeensyInterfaceComponent::subBuzzCb(atl_msgs::msg::Buzzers::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if (msg->data.size() != nBuzz) {
    RCLCPP_ERROR(
      get_logger(), "Buzzer message must be of length 1, received length %lu.",
      msg->data.size());
    throw std::runtime_error("Buzzer message has wrong size.");
  }
  for (std::size_t i = 0; i < nBuzz; i++) {
    buzzMsg_[i] = {msg->data[i], 5};
  }
}

void TeensyInterfaceComponent::subPanelLightsCb(atl_msgs::msg::Lights::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if (msg->data.size() != npl) {
    RCLCPP_ERROR(
      get_logger(), "Panel lights message must be of length %lu, received length %lu.",
      npl, msg->data.size());
    throw std::runtime_error("Panel lights message has wrong size.");
  }
  panelLightsMsg_ = std::move(msg);
}

void TeensyInterfaceComponent::subHMLightsCb(atl_msgs::msg::Lights::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if (msg->data.size() != nhml) {
    RCLCPP_ERROR(
      get_logger(), "Helm Master lights message must be of length %lu, received length %lu.",
      nhml, msg->data.size());
    throw std::runtime_error("Helm Master lights message has wrong size.");
  }
  HMLightsMsg_ = std::move(msg);
}

void TeensyInterfaceComponent::subEngineTrimCb(atl_msgs::msg::TrimsInput::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if (msg->inputs.size() != prm_.n_engines) {
    RCLCPP_ERROR(
      get_logger(), "Engines trim message must have %lu inputs, received %lu.",
      prm_.n_engines, msg->inputs.size());
    throw std::runtime_error("Engines trim message has wrong number of inputs.");
  }

  // if user trim input is non-zero, it is the true trim value
  for (std::size_t i = 0; i < msg->inputs.size(); i++) {
    desTrimInput_[i] = msg->inputs[i];
  }

  engineTrimMsg_ = std::move(msg);
}

void TeensyInterfaceComponent::subTrimTabsCb(atl_msgs::msg::TrimsInput::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if (msg->inputs.size() != nTrimTabs) {
    RCLCPP_ERROR(
      get_logger(), "Trim tabs message must be of length %lu, received length %lu.",
      nhml, msg->inputs.size());
    throw std::runtime_error("Trim tabs message has wrong size.");
  }
  trimTabsMsg_ = std::move(msg);
}

void TeensyInterfaceComponent::subEnginePowerCb(atl_msgs::msg::EnginesPowerInput::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);
  if ((!msg->on.empty() && *std::max_element(msg->on.cbegin(), msg->on.cend()) >= prm_.n_engines) ||
    (!msg->off.empty() && *std::max_element(msg->off.cbegin(), msg->off.cend()) >= prm_.n_engines))
  {
    RCLCPP_ERROR(
      get_logger(), "Engines power message contains an index greater than the number of engines.");
    throw std::runtime_error("Engines power message inconsistency.");
  }

  for (const auto & id : msg->on) {
    enginePowerData_[id]++;
  }

  for (const auto & id : msg->off) {
    enginePowerData_[id]--;
  }
}

}  // namespace atl

RCLCPP_COMPONENTS_REGISTER_NODE(atl::TeensyInterfaceComponent)
