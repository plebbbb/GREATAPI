#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "universal_generator.hpp"

#ifndef WAYPOINTGEN_BEZIER_HPP
#define WAYPOINTGEN_BEZIER_HPP

namespace greatapi {
  struct Bezier : public WaypointGenerator{
    std::vector<coord> params;
    Bezier(std::vector<coord> dataset):params(dataset),WaypointGenerator(){}
      std::vector<coord> compute(int density){
        std::vector<coord> returnset = {};
        for(double t = 0; t <= double(1); t+=(1/double(density))){
          coord tmp = coord();
          for (int i = 0; i < params.size(); i++){
              double ccfactor = (Ptriangle[params.size()-1][i])*pow((1-t),params.size()-1-i)*pow(t,i);
              tmp.x+=ccfactor*params[i].x;
              tmp.y+=ccfactor*params[i].y;
          }
          returnset.emplace_back(tmp);
        }
        return returnset;
      }
  };
}

#endif
