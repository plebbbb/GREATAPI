#pragma once
#include "api.h"

#ifndef SUBFUNCTION_HPP
#define SUBFUNCTION_HPP

namespace greatapi {
  namespace math {
    struct subfunction{
      std::vector<double> params;
      subfunction(std::vector<double> p):params(p){}
      virtual double compute(double in) = 0;
      //integrate() is not designed to be used directly in a subfunction. +C is only implemented in the function integrate function
      virtual subfunction* integrate() = 0;
      virtual subfunction* derive() = 0;
    };
  }
}
#endif
