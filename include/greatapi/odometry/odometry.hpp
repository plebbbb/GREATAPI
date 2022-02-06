//A header guard
#pragma once
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/sensors/twheel.hpp"
#include "greatapi/odometry/rotation_odom.hpp"
#include "main.h"

#ifndef ODOM_FULL_HPP
#define ODOM_FULL_HPP

//attempted odometry rewrite
namespace greatapi{
  namespace odometry{
    /*
      This struct operates on a standard coordinate grid relative to the bot.
      This means that positive Y is the axis that is 90 degrees counterclockwise from positive X.
      Angles start from zero at the positive X axis, and increase in the counterclockwise direction.
      You need an encoder measuring each of these axises, making sure that both of them are positive when moving in the positive direction of the bot
    */
    struct odometry {
      TWheel* Xaxis;
      TWheel* Yaxis;
      distance X_toCOR; // distance of X axis tracking wheel to center of rotation, right is positive //TBD: figure out when measurement is positive/negative(left or right encoder)
      distance Y_toCOR; // distance of Y axis tracking wheel to center of rotation, forwards is positive
      odom_rotation* rotationcalc;
      double encoderangoffset; //angle between the forwards direction of the bot and encoder measured positive Y axis.
      double globaloffset; //angle between forwards direction and the local X axis(positive direction) at bot start position.
      distance Xlast = 0;
      distance Ylast = 0;
      /*
        explaining encoderangoffset and globaloffset

        the heading which the global coordinates use is measured as the angle between the forwards direction of the bot to the global X axis
        in order to rotate the local coordinate grid to the global axis, we need to rotate by the angle between pairs of local axises and global axises
        so that both axises are parallel with each other. This would let us add the coordinates to each other. We can apply an offset to the heading
        indicated by the global coordinates to get the angle between local and global coordinate grids.

        globaloffset does this for us, as it's the offset from the forwards direction of the bot to the local X axis, which gets the angle between global and local

        there is also the edge case where the local Y axis isnt alligned with the forwards direction. Globaloffset doesn't account for this
        so we can add another variable which accounts for the Y axis being offset from the forwards direction.

        This variable is encoderangoffset.
      */

      //NOTE: your all params should be constructed with the new keyword. This is because you need a static memory location for polymorphisism to work.
      odometry(TWheel* X, distance X_to_ctr, TWheel* Y, distance Y_to_ctr, odom_rotation* rotation){
        Xaxis = X;
        Yaxis = Y;
        rotationcalc = rotation;
        //default setup, assumes that tracking wheels are parallel with forwards direction, and that the back of the bot is aganist the X axis wall
        encoderangoffset = 0;
        globaloffset = 0;
        X_toCOR = X_to_ctr;
        Y_toCOR = Y_to_ctr;
      }

      position calculateposition(position initial){
          //get heading, subtract by previous angle to get relative angle change
          SRAD newang = rotationcalc -> get_heading();
          double relAngleChange = findDiff(initial.angle, newang);

          //local coordinate object to be used for transforms
          coord localcoordinate = std::pair<distance,distance>{0,0};

          distance Xtravel = Xaxis -> get_distance() - Xlast; //the distance sensors return sum values. We need net change from previous iteration.
          distance Ytravel = Yaxis -> get_distance() - Ylast;

          Ylast+=Ytravel;
          Xlast+=Xtravel;


          //if no angle change, just add coords
          if(relAngleChange == angle(0)){
            localcoordinate.x += Xtravel;
            localcoordinate.y += Ytravel;
          }

          //if angle change, get arc lengths and transform
          else {
            localcoordinate.y = double(2.0*sin(relAngleChange/2) *
            (((double)Ytravel/relAngleChange) + Y_toCOR));

            localcoordinate.x = double(2.0*sin(relAngleChange/2) *
            (((double)Xtravel/relAngleChange) + X_toCOR));

            localcoordinate = localcoordinate.transform_matrix(angle(-((double)initial.angle+(relAngleChange/2) - globaloffset + encoderangoffset)));
          }

          //update initial position with new coordinate and angle
          initial += localcoordinate;
          initial.angle = newang;

          return initial;
        }
    };
  }
}

#endif
