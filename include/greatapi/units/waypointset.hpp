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
    waypointset(WaypointGenerator e, int density):vector<coord>(e.compute(density)){}
  };

}

#endif
