#include<cmath>
#include "greatapi/units/universal_datatype.hpp"

#ifndef ANGLE_HPP
#define ANGLE_HPP

namespace greatapi {
  //struct angle holds an angluar value in radians
  struct angle : public unit{
    angle():unit(0){};
    angle(double v):unit(v){};

    void operator+=(angle increment) {
      value += (double)increment;
    }

    void operator-=(angle increment) {
      value -= (double)increment;
    }

    void operator/=(angle increment) {
      value /= (double)increment;
    }

    void operator*=(angle increment) {
      value *= (double)increment;
    }

    angle operator+(angle b) {
      return angle(value + b);
    }

    angle operator-(angle b) {
      return angle(value - b);
    }

    angle operator/(angle b) {
      return angle(value / b);
    }

    angle operator*(angle b) {
      return angle(value * b);
    }
  };

  struct degrees : public angle{
    degrees(double v):angle(v*M_PI/180){};
  };

  struct radians : public angle{
    radians(double v):angle(v){};
  };

}

#endif
