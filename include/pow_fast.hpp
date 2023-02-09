#ifndef TEENSY_INTERFACE__POW_FAST_HPP_
#define TEENSY_INTERFACE__POW_FAST_HPP_

#include <cstdint>
#include <type_traits>

namespace atl
{

/***************************************************************************
 * \brief Computes b^e
 ***************************************************************************/
template<typename TBase, typename TExp>
constexpr TBase powFast(const TBase b, const TExp e) noexcept
{
  static_assert(std::is_unsigned_v<TExp>, "Exponent type must be unsigned.");

  if (e == 0) {
    return TBase(1.);
  }

  if (e == 1) {
    return b;
  }

  TBase out = b;
  for (TExp i = 2; i <= e; i++) {
    out *= b;
  }

  return out;
}


// /***************************************************************************
//  * \brief Computes b^e
//  ***************************************************************************/
// template<std::size_t e = 2, typename T>
// constexpr T powFast(const T b) noexcept
// {
//   if constexpr (e == 0) {
//     return T(1);
//   }

//   if constexpr (e == 1) {
//     return b;
//   }

//   T out{b};
//   for (std::size_t i = 2; i <= e; i++) {
//     out *= b;
//   }
//   return out;
// }


namespace detail
{

template<typename T, std::size_t ... Is>
constexpr T powFastImpl([[maybe_unused]] T val, std::index_sequence<Is...>)
{
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-value"
  // *INDENT-OFF*
  return ((Is, val) * ... * T{1});
  // *INDENT-ON*
  #pragma GCC diagnostic pop
}

}  // namespace detail

// /***************************************************************************
//  * \brief Computes b^e via expansion to e-length product  b * ... * b
//  ***************************************************************************/
template<std::size_t e = 2, typename TBase>
constexpr TBase powFast(const TBase b) noexcept
{
  return detail::powFastImpl(b, std::make_index_sequence<e>{});
}

}  // namespace atl

#endif  // TEENSY_INTERFACE__POW_FAST_HPP_
