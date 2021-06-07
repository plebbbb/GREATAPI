//automatic compile time angle unit conversion
#include<cmath>
#include "greatapi/units/angle_units/universal_angle.hpp"
#include "greatapi/units/angle_units/srad.hpp"

#pragma once


#ifndef ANGLE_DIFF_HPP
#define ANGLE_DIFF_HPP

namespace greatapi{
  // When given two smart radians a and b, returns the difference b - a expressed as a radians diff in the range [-PI, PI)
  extern radians findDiff(SRAD a, SRAD b) {
      double diff(b.value);
      diff -= a.value;

      if (diff >= M_PI) {
        diff -= M_PI * 2;
      }
      if (diff < M_PI * (-1)) {
        diff += M_PI * 2;
      }

      return diff;
    }
}

#endif
