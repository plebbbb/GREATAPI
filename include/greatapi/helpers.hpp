#ifndef HELPER_HPP
#define HELPER_HPP

#include <type_traits>
 
#define PI 3.14159265358979323846

 
 template <typename T> inline constexpr
 int signum(T x, std::false_type is_signed) {
     return T(0) < x;
 }

 template <typename T> inline constexpr
 int signum(T x, std::true_type is_signed) {
     return (T(0) < x) - (x < T(0));
 }

 template <typename T> inline constexpr
 int signum(T x) {
     return signum(x, std::is_signed<T>());
 }

 #endif