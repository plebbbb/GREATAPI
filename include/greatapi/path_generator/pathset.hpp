#include <cmath>
#include "greatapi/basic_datatypes.hpp"
#include "waypoint_generator/universal_generator.hpp"

#ifndef PATHSET_HPP
#define PATHSET_HPP

namespace greatapi {
  /*I wanted some clean direct math to position system, but that's too much effort. This is just a wrapper now.

  NOTE: the angles of the position entities inside pathset represent the tangent angle to the travel path.
  If you have a holonomic drive and want to face a different direction, overwrite this angle after obtaining a value here,
  and prior to calling your motor drive functions each loop*/
  struct pathset{
    std::vector<position> waypoints = {};
    int len;
    int lastarrpos = 0;
    pathset(std::vector<std::vector<position>> points){
      //append all waypoints to a single, unified array.
      for (int i = 0; i < points.size(); i++){
        waypoints.insert(waypoints.end(), points[i].begin(), points[i].end());
      }
      len = waypoints.size();
    }
    /*The original intent was that we could get fresh orientation and position data directly from the coordinate
     generation algorithm. The problem was that identifying what's considered our position down the generated path
     is pretty annoying, especially with preventing overlapping paths from causing the bot to skip part of the path,
     as well as defining how we would obtain the exact position down the generated path given that we aren't going to be
     perfectly in line. This is a compromise solution. We identify the nearest coordinate within N points of the
     last recorded position and assume that's probably the right point and we didnt just skip any significant movement*/
     /*
      D_threshold - minimum distance to current location for point acceptability
      lookahead_old - maximum amount of allowed steps ahead for identifying the nearest coordinate which hasn't been targeted
      lookahead_new - maximum amount of allowed steps ahead for identifying a new target point
      just returns the farthest location lookahead_new allows if nothing closer is availiable.

      this function should not be spammed! logic for determining if you need a new target is to be done externally
     */
    position get_next_position(position currentposition, distance D_threshold, int lookahead_new, int lookahead_old){
      int start_index = compute_nearest_coord(currentposition,lookahead_old); //I suggest lookahead_old be the lookahead_new used to get the last target position +1 or +2
      for (int i = start_index; i < std::min(start_index+lookahead_new, len); i++){
        if (coord(currentposition,waypoints[i]).get_length() > D_threshold){
          return waypoints[i];
        }
      }
      return waypoints[std::min(start_index+lookahead_new, len)-1]; //If no points are sufficiently far away
    }

    //computes nearest coord by distance to coordinates after last known used target coordinate
    int compute_nearest_coord(coord position, int lookahead){
      int bestindex = lastarrpos;
      for (int i = lastarrpos; i < std::min(lastarrpos+lookahead, len); i++){
        if (coord(position,waypoints[i]).get_length() > coord(position,waypoints[bestindex]).get_length()){
          bestindex = i;
        }
      }
      lastarrpos = bestindex;
      return bestindex;
    }

    position operator[](int iter){
      return waypoints[iter];
    }

  };
}
#endif
