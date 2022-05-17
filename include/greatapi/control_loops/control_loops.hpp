//A header guard
#pragma once
#include "api.h"
#include "greatapi/basic_datatypes.hpp"

#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

namespace greatapi{
  //if anyone wants to try, it would be nice if we could implicitly construct these. It would require a restructuring of everything though.
  struct controlelement{
      double factor;
      controlelement(double fac):factor(fac){}
      virtual double compute(double target, double current) = 0;
  };

  struct Proportional: public controlelement{
      double maxcap;
      double mincap;
      Proportional(double fac):controlelement(fac){
        mincap = __DBL_MIN__;
        maxcap = __DBL_MAX__;
      }

      Proportional(double fac, std::pair<double,double> caps):controlelement(fac){
          maxcap = std::get<0>(caps);
          mincap = std::get<1>(caps);
      }

      //standard offset format: target-current. This class assumes the offset is in the correct direction already
      double compute(double target, double current){
          double rawval = (target-current);
          double returnval = factor*rawval;
          return (returnval <= maxcap) ? ((returnval >= mincap) ? returnval : mincap) : maxcap;
      }
  };

  struct Integral: public controlelement{
      double last = 0;
      double maxcap;
      double mincap;

      //this is NOT recommended due to integral windup
      Integral(double fac):controlelement(fac){
        mincap = __DBL_MIN__;
        maxcap = __DBL_MAX__;
      }

      Integral(double fac, std::pair<double,double> caps):controlelement(fac){
          maxcap = std::get<0>(caps);
          mincap = std::get<1>(caps);
      }

      //standard offset format: target-current. This class assumes the offset is in the correct direction already
      double compute(double target, double current){
          if((int)target == (int)current) last = 0;
          double rawval = (target-current);
          last += rawval;
          double returnval = last*factor;
          return (returnval <= maxcap) ? ((returnval >= mincap) ? returnval : mincap) : maxcap;
      }
  };

  struct Derivative: public controlelement{
      double past = 0;
      double maxcap;
      double mincap;
      Derivative(double fac):controlelement(fac){
        mincap = __DBL_MIN__;
        maxcap = __DBL_MAX__;
      }

      Derivative(double fac, std::pair<double,double> caps):controlelement(fac){
          maxcap = std::get<0>(caps);
          mincap = std::get<1>(caps);
      }

      //standard offset format: target-current. This class assumes the offset is in the correct direction already
      double compute(double target, double current){
          double rawval = target-current;
          double returnval = factor * (rawval-past);
          past = rawval;
          return (returnval <= maxcap) ? ((returnval >= mincap) ? returnval : mincap) : maxcap;
      }
  };

  //Modular control loop, computes values for a set of control elements
  struct control_loop{
      std::vector<controlelement*> elementset;
      double maxcap;
      double mincap;
      //if you dont like caps just set them really high, like +-INT_MAX or something
      control_loop(std::vector<controlelement*> val, std::pair<double,double> caps):elementset(val){
          maxcap = std::get<0>(caps);
          mincap = std::get<1>(caps);
      }

      //alternate constructor, assumes min and max caps are int limit.
      control_loop(std::vector<controlelement*> val):elementset(val){
          maxcap = 2147483647;
          mincap = -2147483647;
      }

      double update(double target, double current){
          double returnval = 0;
          //no enhanced for to stop sketchy copying issues
          for (int i = 0; i < elementset.size(); i++){
              returnval += elementset[i]->compute(target,current);
          }
          return (returnval <= maxcap) ? ((returnval >= mincap) ? returnval : mincap) : maxcap;
      }
  };
};


#endif
