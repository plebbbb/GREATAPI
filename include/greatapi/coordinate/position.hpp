//combined position datatype. Combines SRAD and Coord to produce a robot position
#include "greatapi/angle_units/srad.hpp"
#include "greatapi/coordinate/coord.hpp"
#pragma once
#ifndef POSITION_HPP
#define POSITION_HPP

namespace greatapi{

  //combines an angle and coordinate,
  struct position{
    coord location;
    SRAD angle;
    position(coord vector, SRAD ang):location(vector),angle(ang){};

    /******************************************************************************/
    //Conversion functions
    operator coord(){
      return location;
    }

    operator SRAD(){
      return angle;
    }

    void operator+=(coord change) {
      location+=change;
      location.get_length();
    }

  };

}

#endif
