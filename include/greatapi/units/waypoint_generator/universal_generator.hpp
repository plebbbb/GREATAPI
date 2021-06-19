#include<cmath>
#include "greatapi/basic_datatypes.hpp"

#ifndef WAYPOINTGEN_HPP
#define WAYPOINTGEN_HPP

namespace greatapi {
  struct WaypointGenerator{
    //generates coord array of pointcount points according to some parametric function
    virtual std::vector<coord> compute(int pointcount);

    //completion ranges from 0 to 1.
    virtual SRAD computeheading(double completion);

    virtual double computeslope(double completion);
  };
}

#endif
