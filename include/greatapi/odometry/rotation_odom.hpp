//A header guard
#pragma once
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/sensors/twheel.hpp"
#include "main.h"

#ifndef ROT_ODOM_HPP
#define ROT_ODOM_HPP

//attempted odometry rewrite
namespace greatapi{
  namespace odometry{

    //parent class of rotational odometry systems
    struct odom_rotation{
      double DOffset = 0; //drift offset. This is a hard offset which is designed to be used for on-the-fly odom resets at known headings. this is in radians.
      odom_rotation(){}

      virtual SRAD get_heading_raw() = 0; //pure virtual function. Defined by each odometry system.

      //gets raw heading and applies calibration offsets to it.
      SRAD get_heading(){
        return SRAD(get_heading_raw() + DOffset);
      }

      //sets new DOffset based on the difference between the real angle and the IMU reported angle.
      void applyOffset(SRAD real){
        DOffset = findDiff(get_heading_raw(),real); //subtracts getHeading() from real angle. Thus, Doffset+getHeading = real angle.
      }

      virtual void calibrate() = 0;
      virtual void tare() = 0;
    };

    //single inertial sensor rotation system
    struct IMU_odom_rotation : public odom_rotation{
      pros::Imu Inertial;
      double DFC; //drift compensation factor, usually about 101-105% of returned rotation value to account for integration drift
      /*
        DFC can be calculated by spinning your bot a known angle(usually a lot, like 20 spins or so to minimize inaccuracies),
        and then comparing the IMU angle and the real angle. Dividing the known angle by the IMU angle yields you the DFC.
        It is recommended to rotate in 360 degree intervals for your rotations to make it easier to determine the angle.
        Eyeballing is probably safe after for a known angle after sufficient spins.
      */

      //no DFC(drift compensation factor) constructor
      IMU_odom_rotation(int port):Inertial(port),DFC(double(1)),odom_rotation(){};

      //DFC(drift compensation factor) constructor
      IMU_odom_rotation(int port, double driftcompensationfac):Inertial(port),DFC(driftcompensationfac),odom_rotation(){};

      SRAD get_heading_raw() {
        return SRAD(degrees(((double)Inertial.get_heading())*DFC)); //turn raw angle into radians, then constrain to 0-2PI interval via SRAD constructor
      }

      void calibrate(){
        Inertial.reset();
        pros::delay(2100);
      }

      void tare(){
        Inertial.tare();
      }

    };

/* commented out because not implemented yet. 

    //double inertial sensor system
    struct DoubleIMU_odom_rotation : public odom_rotation{
      pros::Imu Positive;
      pros::Imu Negative;
      DoubleIMU_odom_rotation(int pos, int neg):Positive(pos),Negative(neg),odom_rotation(){};

      SRAD get_heading_raw(){
        //put heading calcs here plz.
      }
    }; //To get angle, atan2 the raw X and Y values you get or something and add that to existing angle. Idk how u do this backwards for averaging tho.
*/

    //ADI encoder(the big red ones) system
    struct TWheel_odom_rotation : public odom_rotation{
      TWheel* Left; //pointer for polymorphisism between ADI encoders and new V5 rotation sensor tracking wheels
      TWheel* Right;
      distance rotationalDist; //measured distances between the tracking wheels, perpendicular to their normal rotation direction.

      //NOTE: you should be constructing TWheels with the new keyword, so that they exist in their own memory space.
      TWheel_odom_rotation(TWheel* L, TWheel* R, distance dist_btwn):rotationalDist(dist_btwn),odom_rotation(){
        Left = L;
        Right = R;
      };

      SRAD get_heading_raw(){
        return SRAD(double((Right->get_distance() - Left->get_distance())/rotationalDist));
      }

      void calibrate(){
        
      }
      void tare(){
        Left->reset();
        Right->reset();
        //pros::delay(5);
      }
    };
  }
}

#endif
