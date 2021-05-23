//combined position datatype. Combines SRAD and Coord to produce a robot position
#include "greatapi/angle_units/srad.hpp"
#include "greatapi/coordinate/coord.hpp"
#pragma once
#ifndef POSITION_HPP
#define POSITION_HPP

//combines the two
struct Position{
  coord location;
  SRAD angle;
  Position(coord vector, SRAD ang):location(vector),angle(ang){};

  /******************************************************************************/
  //Conversion functions
  operator Coord(){
    return location;
  }

  operator SRAD(){
    return angle;
  }

  void operator+=(Coord change) {
    location+=change;
    get_length();
  }

}



#endif
