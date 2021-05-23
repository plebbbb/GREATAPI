//automatic compile time angle unit conversion
#include<cmath>
constexpr double operator"" _radians(long double x) {
  return double(x);
}

constexpr double operator"" _degrees(long double x) {
  return double(x)*M_PI/180;
}

extern double DegToRad(double x){
  return x*M_PI/180;
}
