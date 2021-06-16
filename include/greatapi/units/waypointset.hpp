#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/units/waypoint_generator/universal_generator.hpp"
#include "greatapi/units/waypoint_generator/bezier.hpp"

#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

namespace greatapi {
  //apparently this is legal?
  //ngl, not sure if we want to integrate PP support directly into this
  struct waypointset : public std::vector<coord> {
    waypointset(std::vector<coord> coordinateset):vector<coord>(coordinateset){}

    //X has an interval between 0 and 1.
    virtual SRAD get_slope(double t);
  };

  struct Gwaypointset : public waypointset {
    WaypointGenerator generator;
    Gwaypointset(WaypointGenerator e, int density):waypointset(e.compute(density)),generator(e){}

    SRAD get_slope(double t){
      return generator.computeheading(t);
    }
  };

}

#endif
