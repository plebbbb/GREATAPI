#include<cmath>
#include "greatapi/units/universal_datatype.hpp"

#ifndef DISTANCE_HPP
#define DISTANCE_HPP

namespace greatapi {
  //standard distance units are INCHES
  struct distance : public unit{
    distance():unit(0){};
    distance(double v):unit(v){};

    void operator+=(distance increment) {
      value += increment.value;
    }

    void operator-=(distance increment) {
      value -= increment.value;
    }

    void operator/=(distance increment) {
      value /= increment.value;
    }

    void operator*=(distance increment) {
      value *= increment.value;
    }

    distance operator+(distance b) {
      return distance(value + b.value);
    }

    distance operator-(distance b) {
      return distance(value - b.value);
    }

    distance operator/(distance b) {
      return distance(value / b.value);
    }

    distance operator*(distance b) {
      return distance(value * b.value);
    }
  };

  struct centimeters : public distance{
    centimeters(double v):distance(v / 2.54){};
  };

  struct meters : public centimeters{
    meters(double v):centimeters(v/1000){};
  };

  struct inches : public distance{
    inches(double v):distance(v){};
  };

}

#endif
