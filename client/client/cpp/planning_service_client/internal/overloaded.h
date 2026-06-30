#pragma once

// The "overloaded" variant-visit pattern.

namespace {

/**
 *   using MyVariant = std::variant<int, std::string>;
 *   MyVariant v = 5;
 *   std::string result = std::visit<const char*>(overloaded{
 *     [](const int arg) { return "found an int"; },
 *     [](const std::string& arg) { return "found a string"; }
 *   }, v);
 *
 */
// Boilerplate for `std::visit`; see
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};

template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace
