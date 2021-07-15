#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/units/waypoint_generator/universal_generator.hpp"
#include "greatapi/units/waypoint_generator/bezier.hpp"
#include "greatapi/units/waypointset.hpp"

#ifndef PATHSET_HPP
#define PATHSET_HPP

namespace greatapi {
  //wrapper for sets of waypointsets. Done as to simplify percentage completion calulations
  struct pathset {
    std::vector<waypointset*> locations; //pointer for polymorphism
    std::vector<inches> waypointD;
    inches SV = 0; //sum traveled distance
    pathset(std::vector<waypointset*> set): locations(set){
      for(waypointset* e : locations){
        waypointD.emplace_back(computedist(e));
        SV+=waypointD[waypointD.size()-1];
      }
    }

    //WARNING: DO NOT PUT A C ARRAY IN HERE
    pathset append(waypointset* NV){
      waypointD.emplace_back(computedist(NV));
      SV+=waypointD[waypointD.size()-1];
      locations.emplace_back(NV);
      return *this;
    }

    //THIS ONE ADDS SETS OF WAYPOINTS
    pathset append(std::vector<waypointset*> add){
      for(waypointset* NV : add){
        waypointD.emplace_back(computedist(NV));
        SV+=waypointD[waypointD.size()-1];
        locations.emplace_back(NV);
      }
      return *this;
    }

    //THIS ONE ADDS SETS OF WAYPOINTS
    pathset append(pathset add){
      for(waypointset* NV : add.locations){
        locations.emplace_back(NV);
      }
      for(inches ND : add.waypointD){
        waypointD.emplace_back(ND);
      }
      SV += add.SV;
      return *this;
    }

    //vague aproxmiation of traveled distance
    inches computedist(waypointset* given){
      inches SD = 0; //length sum for specific waypointset
      for(int i = 1; i < given -> size(); i++){ //seems semi risky, probably works
        //somewhat concerned with performance issues, but this should be done at startup and point resolution shouldnt be too high anyways
        SD += (given->at(i) - given->at(i-1)).length; //subtract two coords and append computed length to length sum
      }
      return SD;
    }

    //completion is a value of interval 0 - 1, in percent
    coord get(double completion){
      double RELPC;
      double SM = 0;
      for(int i = 0; i < locations.size(); i++){
        SM+=waypointD[i];
        double MAX = SM/SV;
        if(MAX > completion){
          RELPC = (MAX - completion)/(waypointD[i]/SV); //divide local % completion by total % completion of said waypointset to get %compl of that waypointset
          return locations.at(i) -> getbyperc(RELPC); //get coord coorsponding to the % completion.
        }
      }
    }
  };
}
#endif
