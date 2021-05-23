/*
SMART RADIANS
This is a class providing a self-constraining radian datatype.
*/
#include "api.h"
#pragma once

#ifndef SRAD_HPP
#define SRAD_HPP

/*A smart radian automatically prunes all values to be within the range of 0 - 2PI
Its a radian, but smart*/
struct SRAD {
  double value;
  /******************************************************************************/
  //Constructors:
  SRAD() { value = 0; } //default constructor, if you don't make it anything it's 0 by default

  SRAD(double angle_in_radians) {
    value = angle_in_radians;
    prune();
  }

  // When given two smart radians a and b, returns the difference b - a expressed as a radians diff in the range [-PI, PI)
  double findDiff(SRAD a, SRAD b) {
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

  SRAD operator=(double angle_in_radians) {
    return SRAD(angle_in_radians);
  }


  /******************************************************************************/
  //Utility functions

  void prune() { //constrains the angle values into the range of 0 - 2PI

    //Division by zero correction is to set angle to 0
    if (isnanf(value) || isinff(value)) value = 0;

    //C++ fmod is actually just a remainder function, we have to do this
    //to conform to normal modulus rules, where there cannot be negative values
    value = fmod(fmod(value,2*M_PI)+2*M_PI,2*M_PI);
  //	value = fmod(value,2*M_PI);
  }

  /******************************************************************************/
  //Conversion functions
  operator double() {
    prune();
    return value;
  }


  /******************************************************************************/
  //Manipulation functions
  void operator+=(SRAD increment) {
    value += (double)increment;
    prune();
  }

  void operator-=(SRAD increment) {
    value -= (double)increment;
    prune();
  }

};

#endif
