#include<cmath>
#include "greatapi/basic_datatypes.hpp"

#ifndef WAYPOINTGEN_HPP
#define WAYPOINTGEN_HPP

namespace greatapi {
  struct WaypointGenerator{
    virtual std::vector<coord> compute(int pointcount);

    //completion ranges from interval of 0 to 100.
    virtual SRAD computeheading(double completion);
  };
}

#endif
