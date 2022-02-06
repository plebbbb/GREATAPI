#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "universal_generator.hpp"

#ifndef WAYPOINTGEN_LINEAR_HPP
#define WAYPOINTGEN_LINEAR_HPP

namespace greatapi {
  struct Lines : public WaypointGenerator{
    std::vector<coord> params;

    Lines(std::vector<coord> dataset):params(dataset),WaypointGenerator(){
    }

    std::vector<position> compute(int density){
      std::vector<position> returnset = {};
      for(double t = 0; t <= double(1); t+=(1/double(density))){
        returnset.emplace_back(position(computePC(params,t), computeheading(t));
      }
      return returnset;
    }

    int get_segement_upper_index(double PC){
      return std::min(ceil(PC*params.size()),params.size()-1);
    }

    //computes a coordinate at given value PC
    coord computePC(std::vector<coord> target, double PC){
      double upper_seg = get_segement_upper_index(PC);
      double distscalefac = (PC - (upper_seg-1)/params.size())/params.size();
      return coord(
        params[upper_seg-1].x+(params[upper_seg].x-params[upper_seg-1].x)*distscalefac,
        params[upper_seg-1].y+(params[upper_seg].y-params[upper_seg-1].y)*distscalefac
      )
    }

    //TBD: we need a reverse solver to determine nearest % completion.
    //this will likely need some sorta newton-rasphaodian solver where we approach the correct % completion with more iterations
    SRAD computeheading(double percCompl){
      double upper_seg = get_segement_upper_index(percCompl);
      coord vector(params[upper_seg-1].x, params[upper_seg]);
      return SRAD(atan2(vector.y, vector.x)); //atan2 converts x and y into angle of interval +PI to -PI. SRAD constructor turns that interval into 0-2PI
    }

    double computeslope(double percCompl){
      double upper_seg = get_segement_upper_index(percCompl);
      coord vector(params[upper_seg-1].x, params[upper_seg]);
      return vector.y/vector.x; //returns rise/run slope
    }
  };
}

#endif
