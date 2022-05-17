#pragma once
#include "api.h"
#include "subfunction/subfunction.hpp"
#include "subfunction/function_list.hpp"

#ifndef FUNCTION_HPP
#define FUNCTION_HPP

namespace greatapi{
  namespace math{
    //TBD: support parametric equations and integrate with bezier curve path generation
    struct function{
      std::vector<subfunction*> terms;
      //TBD: implement varadic constructor for this, think like printf
      function(std::vector<subfunction*> list):terms(list){}

      double compute(double in){
        double rv = 0;
        for (subfunction* a : terms){
            rv += a->compute(in);
        }
        return rv;
      }

      function integrate(double c){
        std::vector<subfunction*> rv;
        rv.push_back(new polynomial({c}));
        for (subfunction* a : terms){
            rv.push_back(a->integrate());
        }
        return rv;
      }

      function derive(){
        std::vector<subfunction*> rv;
        for (subfunction* a : terms){
            rv.push_back(a->derive());
        }
        return rv;
      }

      /******************************************************************************/
      //Conversion functions

      //Implicit conversion for integration of functions inside functions, use STD::insert to merge the vectors
      operator std::vector<subfunction*>(){
        return terms;
      }

      /******************************************************************************/
  		//Manipulation functions

      //Add two functions together
      function operator+(function b){
        std::vector NV = terms;
        NV.insert(NV.end(), b.terms.begin(), b.terms.end());
        return function(NV);
      }
    };
  }
}
#endif
