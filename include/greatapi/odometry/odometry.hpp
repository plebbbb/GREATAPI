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
      double X_toCOR; // distance of X axis tracking wheel to center of rotation
      double Y_toCOR; // distance of Y axis tracking wheel to center of rotation
      odom_rotation* rotationcalc;
      double encoderangoffset; //angle between the forwards direction of the bot and encoder measured positive Y axis. Positive when pos Y axis CCW of forwards direction, otherwise negative
      double globaloffset; //angle between encoder measured positive X axis and the forwards direction of the bot. Positive when forwards is CCW of X axis.

      /*
        explaining forwardsoffset and globaloffset

        the heading which the global coordinates use is measured as the angle between the forwards direction of the bot to the global X axis
        in order to rotate the local coordinate grid to the global axis, we need to rotate by the angle between pairs of local axises and global axises
        so that both axises are parallel with each other. This would let us add the coordinates to each other. We can apply an offset to the heading
        indicated by the global coordinates to get the angle between local and global coordinate grids.

        globaloffset does this for us, as it's the offset from the forwards direction of the bot to the local X axis.

        there is also the edge case where the local Y axis isnt alligned with the forwards direction. Globaloffset doesn't account for this
        so we can add another variable which accounts for the Y axis being offset from the forwards direction.

        This variable is encoderangoffset.
      */

      //NOTE: your all params should be constructed with the new keyword. This is because you need a static memory location for polymorphisism to work.
      odometry(TWheel* X, TWheel* Y, odom_rotation* rotation){
        Xaxis = X;
        Yaxis = Y;
        rotationcalc = rotation;
      }

      position calculateposition(position initial){
          //get heading, subtract by previous angle to get relative angle change
          SRAD newang = rotationcalc -> get_heading();
          double relAngleChange = initial.angle.findDiff(initial.angle, newang);

          //local coordinate object to be used for transforms
          coord localcoordinate = std::pair<double,double>{0,0};

          //if no angle change, just add coords
          if(relAngleChange == 0){
            localcoordinate.x += Xaxis -> get_distance();
            localcoordinate.y += Yaxis -> get_distance();
          }

          //if angle change, get arc lengths and transform
          else {
            localcoordinate.y = double(2.0*sin(relAngleChange/2) *
            ((Yaxis -> get_distance()/relAngleChange) + Y_toCOR));

            localcoordinate.x = double(2.0*sin(relAngleChange/2) *
            ((Xaxis -> get_distance()/relAngleChange) + X_toCOR));

            localcoordinate = localcoordinate.transform_matrix(-((double)initial.angle+(relAngleChange/2) - globaloffset + encoderangoffset));
          }

          //update initial position with new coordinate and angle
          initial.location += localcoordinate;
          initial.angle = newang;

          return initial;
        }
    };
  }
}

#endif
