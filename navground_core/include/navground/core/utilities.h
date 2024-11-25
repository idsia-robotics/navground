#ifndef NAVGROUND_CORE_UTILITIES_H
#define NAVGROUND_CORE_UTILITIES_H

#include "navground/core/common.h"
#include "navground/core/types.h"
#include <array>
#include <string>
#include <string_view>
#include <vector>

template <typename T> constexpr auto get_type_name() -> std::string_view {
#if defined(__clang__)
  constexpr auto prefix = std::string_view{"[T = "};
  constexpr auto suffix = "]";
  constexpr auto function = std::string_view{__PRETTY_FUNCTION__};
#elif defined(__GNUC__)
  constexpr auto prefix = std::string_view{"with T = "};
  constexpr auto suffix = "; ";
  constexpr auto function = std::string_view{__PRETTY_FUNCTION__};
#elif defined(_MSC_VER)
  constexpr auto prefix = std::string_view{"get_type_name<"};
  constexpr auto suffix = ">(void)";
  constexpr auto function = std::string_view{__FUNCSIG__};
#else
#error Unsupported compiler
#endif

  const auto start = function.find(prefix) + prefix.size();
  const auto end = function.find(suffix);
  const auto size = end - start;

  return function.substr(start, size);
}

template <>
constexpr auto get_type_name<navground::core::Vector2>() -> std::string_view {
  return "Vector2";
}

template <>
constexpr auto get_type_name<std::vector<navground::core::Vector2>>()
    -> std::string_view {
  return "std::vector<Vector2>";
}

#endif // NAVGROUND_CORE_UTILITIES_H
