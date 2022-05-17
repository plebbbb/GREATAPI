#include <cmath>
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/waypoints.hpp"
#include "greatapi/control_loops/control_loops.hpp"

#ifndef PPCONTROLLER_HPP
#define PPCONTROLLER_HPP

namespace greatapi {
  struct PPController{
    control_loop angle;
    control_loop velocity;
    pathset* travelpath;
    //TBD: check if we can copy-initialize the control loops here, it not, turn angle and velocity into pointers
    PPController(control_loop heading_controller, control_loop velocity_controller):angle(heading_controller),velocity(velocity_controller){
    }

    std::pair<SRAD,inches> compute_next_trajectory(position current)
  };
}
#endif
