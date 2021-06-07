//combined position datatype. Combines SRAD and Coord to produce a robot position
#include "greatapi/units/angle_units/srad.hpp"
#include "greatapi/units/coordinate/coord.hpp"
#pragma once
#ifndef POSITION_HPP
#define POSITION_HPP

namespace greatapi{

  //combines an angle and coordinate,
  struct position : public coord{
    SRAD angle;
    position(coord vector, SRAD ang):coord(vector),angle(ang){};

    /******************************************************************************/
    //Conversion functions
    operator SRAD(){
      return angle;
    }
  };

}

#endif
