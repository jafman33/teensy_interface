# mab_inteface

Bridge between ROS and the Teensy 4.1 mc-board

## Published topics

<!-- * ```boat_mode``` of type ```bps_msgs/Mode``` : Controller mode change request from the MAB -->

<!-- * ```engine_state``` of type ```bps_msgs/EngineState``` : Current state of the engines (rpm, steering, clutch) -->

<!-- - `joystick` of type `atl_msgs/Joy` : Current state of boat joysticks and buttons -->

## Subscribed topics

- `input` of type `atl_msgs/ServoInput` : Inputs to the servos

## Parameters

- `teensy_ip` of type `string` : IP address of the MAB
  - Read-only parameter
  - Default: `192.168.1.3`

* `send_port` of type `int64` : Port the UDP packets addressed to the MAB are sent to

  - Read-only parameter
  - Default: `1560`

* `receive_port` of type `int64` : Port on which the node is listening for UDP packets from the MAB
  - Read-only parameter
  - Default: `1561`

- `max_udp_receive_len` of type `int64` : Maximum expected size (in bytes) of the UDP packets from the MAB

  - Read-only parameter
  - Default: `1024`

- `publish_joy` of type `bool` : Whether or not the `joystick` topic is being published
  - Default: `true`

## Behavior

This node relays the UDP messages to and from the Teensy 4.1 mc-board onto the ROS network.

Input messages are forwarded to the Teensy only if their `mode` field is `RUN` (has an integer value of 1).
