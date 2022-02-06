#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "universal_generator.hpp"

#ifndef WAYPOINTGEN_BEZIER_HPP
#define WAYPOINTGEN_BEZIER_HPP

namespace greatapi {
  struct Bezier : public WaypointGenerator{
    std::vector<coord> params;

    //the slope of a bezier curve at state Z is the rise/run of the bezier curve of the differences between each point in the OG bezier
    std::vector<coord> slopeparams = {};

    Bezier(std::vector<coord> dataset):params(dataset),WaypointGenerator(){
      for(int i = 0; i < params.size()-2; i++){
        slopeparams.emplace_back(coord(params[i+1].x - params[i].x, params[i+1].y - params[i].y));
      }
    }

    std::vector<position> compute(int density){
      std::vector<position> returnset = {};
      for(double t = 0; t <= double(1); t+=(1/double(density))){
        returnset.emplace_back(position(computePC(params,t), computeheading(t));
      }
      return returnset;
    }

    //computes a coordinate at given value PC
    coord computePC(std::vector<coord> target, double PC){
      coord tmp = coord();
      for (int i = 0; i < target.size(); i++){
          double ccfactor = (Ptriangle[target.size()-1][i])*pow((1-PC),target.size()-1-i)*pow(PC,i);
          tmp.x+=ccfactor*target[i].x;
          tmp.y+=ccfactor*target[i].y;
      }
      return tmp;
    }

    //TBD: we need a reverse solver to determine nearest % completion.
    //this will likely need some sorta newton-rasphaodian solver where we approach the correct % completion with more iterations
    SRAD computeheading(double percCompl){
      coord vector = computePC(slopeparams, percCompl); //the magnitude of this heading is irrelevant, I think. we need to turn this slope into an angle.
      return SRAD(atan2(vector.y, vector.x)); //atan2 converts x and y into angle of interval +PI to -PI. SRAD constructor turns that interval into 0-2PI
    }

    double computeslope(double percCompl){
      coord vector = computePC(slopeparams, percCompl); //the magnitude of this heading is irrelevant, I think. we need to turn this slope into an angle.
      return vector.y/vector.x; //returns rise/run slope
    }
  };
}

#endif
