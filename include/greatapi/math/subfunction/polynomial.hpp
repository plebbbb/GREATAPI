#pragma once
#include "subfunction.hpp"
#include <cmath>

#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

namespace greatapi {
  namespace math {
    struct polynomial: public subfunction{
      polynomial(std::vector<double> p):subfunction(p){}

      double compute(double term){
        double rv = 0;
        for (int i = 0; i < params.size(); i++){
          rv += params[i]*pow(term, i);
        }
        return rv;
      }

      //this can make memory leaks, remember to delete if you spam these
      polynomial* derive(){
        std::vector<double> rv;
        for (double i = 1; i < params.size(); i++){
          rv.push_back(i*params[i]);
        }
        return new polynomial(rv);
      }

      //this can make memory leaks, remember to delete if you spam these
      polynomial* integrate(){
        std::vector<double> rv = {0};
        for (double i = 0; i < params.size(); i++){
          rv.push_back(params[i]/(i+1));
        }
        return new polynomial(rv);
      }
    };
  }
}
#endif
