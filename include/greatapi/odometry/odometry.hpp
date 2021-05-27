//A header guard
#pragma once
#include "greatapi/basic_datatypes.hpp"
#include "api.h"

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

//BTW, this is ripped directly from my old code, pls refactor and seperate into files


namespace greatapi{

//BasicLibrary is what we will call our library.
//To refrence anything from the library, we need to have the name followed by a ::, then the requested item.
//For example, BasicLibrary::Somefunction() would work, Somefunction() would not work.
  struct DeadWheel;
  struct OdometryWheels;
  struct OdometryComputer;

  enum ENCODER_Position{
    ENCODER_Position_LEFT = 0,
    ENCODER_Position_RIGHT = 1,
    ENCODER_Position_BACK = 2
  };

  //wrapper class for two IMUs, whose results are then averaged out
  //NOTE: THIS DOUBLE IMU CODE DOESNT ACTUALLY CANCEL OUT DRIFT. IT ONLY AVERAGES IT. PLS REVISE LATER.
  struct DoubleIMU{
    pros::Imu L; //imu where clockwise rotation yields negative angles
    pros::Imu R; //imu where clockwise rotation yields positive angles
    DoubleIMU(int CCWP, int CWP):L(CCWP),R(CWP){};
    bool is_calibrating(){
      if(!L.is_calibrating() && !R.is_calibrating()) return false;
      return true;
    }
    SRAD get_heading(){
      double rawvalue = DegToRad(-(L.get_rotation()));
      return (double)((rawvalue)+M_PI/2)*1.01056196909; //estimated IMU drift. probably have to be retuned for each IMU. pls add capability to change later.
    }
    SRAD get_heading_AVG(){
      double rawvalue = DegToRad(-(L.get_rotation() + R.get_rotation())/(double(2)));
      return (double)((rawvalue)+M_PI/2);
    }
  };

  //An ADI encoder wrapper that directly outputs distance values
  struct DeadWheel{
    pros::ADIEncoder Encoder; //The shaft encoder
    double WheelRadius; //The radius of the deadwheel
    double Distance_CenterOfRotation; //Distance to center of rotation
    int encoderTotal = 0; // Stores total encoder value

    //Constructor if the encoder is plugged directly into the V5 Brain ADI ports
    DeadWheel(int portA, int portB, bool direction, double Diameter, double Dist_to_ctr):
    Encoder(portA,portB,direction)
    {
      WheelRadius = double((double)Diameter/2.000000);
      Distance_CenterOfRotation = Dist_to_ctr;
    }

    //Constructor if the encoder is plugged into an ADI Expander
    DeadWheel(pros::ext_adi_port_tuple_t portAB, bool direction, double Diameter, double Dist_to_ctr):
    Encoder(portAB,direction)
    {
      WheelRadius = double((double)Diameter/2.000000);
      Distance_CenterOfRotation = Dist_to_ctr;
    }

    //Returns us the raw angle traversed by the encoder in double
    double get_radian(){
      return (DegToRad(Encoder.get_value()));
    }

    //Returns us the effective distance traveled by the attached wheel
    double get_distance(){
      return double((double)WheelRadius*(double)get_radian());
    }

    //get_distance. Note that this resets the encoder as well. Do not call this multiple times per cycle
    double get_distance_AUTORESET(){
      double val = get_distance();
      encoderTotal += get_distance(); // Add encoder's value to a total
      reset();
      return val;
    }

    // Returns radian value of the encoder total
    double get_radianTotal(){
      return DegToRad((encoderTotal));
    }

    // Returns total distance via the encoder total
    double get_distanceTotal(){
      return double((double)WheelRadius*(double)get_radianTotal());
    }

    void reset(){
      Encoder.reset();
    }
  };

  //This is a struct meant to hold the DeadWheels in an convienent to move container
  //Treat it like an array that has built in functions. It can literally do the [] and the {}

  struct OdometryWheels{
    //This struct is constructed with bracket notation.
    //For example, DeadWheel a = {LEFT, RIGHT, REAR};
    DeadWheel LEFT;
    DeadWheel RIGHT;
    DeadWheel BACK;
//    OdometryWheels(DeadWheel L, DeadWheel R, DeadWheel B):LEFT(L),RIGHT(R),BACK(B){}

    //This approach only measures the encoder values once per cycle, ensuring synchronization of measurements
    std::unique_ptr<std::array<double, 3>> get_distances(){
      return std::unique_ptr<std::array<double, 3>>(
        new std::array<double, 3> {
          LEFT.get_distance_AUTORESET(),
          RIGHT.get_distance_AUTORESET(),
          BACK.get_distance_AUTORESET()
        }
      );
    }

    std::array<double,3> get_distances_nonpointer(){
      return std::array<double, 3>{
        LEFT.get_distance_AUTORESET(),
        RIGHT.get_distance_AUTORESET(),
        BACK.get_distance_AUTORESET()
      };
    }

    // Returns array of total encoder distances
    std::array<double,3> get_distancesTotal_nonpointer(){
      return std::array<double, 3>{
        LEFT.get_distanceTotal(),
        RIGHT.get_distanceTotal(),
        BACK.get_distanceTotal()
      };
    }
    /*This function basically takes control of the [] thing you see in arrays.
    This turns the object into a psudo-array thats basically an array.
    Usable values are index 0, 1 2. or 'ENCODER_Position_LEFT', right and center.
    FYI, this doesn't reset, so you will have to manual reset with the reset function.*/
    double DistOf(ENCODER_Position index){
        switch(index){
            case ENCODER_Position_LEFT: return LEFT.get_distance();
            case ENCODER_Position_RIGHT: return RIGHT.get_distance();
            case ENCODER_Position_BACK: return BACK.get_distance();
        };
    }

    DeadWheel operator[] (ENCODER_Position index){
      switch(index){
          case ENCODER_Position_LEFT: return LEFT;
          case ENCODER_Position_RIGHT: return RIGHT;
          case ENCODER_Position_BACK: return BACK;
      };
    }

    void reset(){
      LEFT.reset();
      RIGHT.reset();
      BACK.reset();
    }
  };


  //Odometry class that actually does the calculations
  struct OdometryComputer{
    OdometryWheels wheels;

    /******************************************************************************/
    //Constructors:
    OdometryComputer(OdometryWheels wheelset):wheels(wheelset){}


    /******************************************************************************/
    //Utility functions
    /*The edge case of a division by zero distance is only observed in odometry
    Hence, we have this janky internal function to deal with these situations*/
    double divzerocomp(double numerator, double denomator){
      if (numerator == 0 || denomator == 0) return 0.0;
      return numerator/denomator;
    }



    /******************************************************************************/
    //Primary functions
    //original odometry function. verfied working
    Position cycle(Position precycle){
      std::array EncoderDistanceValues = wheels.get_distances_nonpointer();
      //its a 50/50 that get_distances_nonpointer works

      //Assuming forwards is 0rad, CCW is positive we calculate the relative offset
      //All coords are prior to move fyi.
      double rel_orientation_change =
      (EncoderDistanceValues[0]-EncoderDistanceValues[1]) /
      (wheels[ENCODER_Position_LEFT].Distance_CenterOfRotation +
        wheels[ENCODER_Position_RIGHT].Distance_CenterOfRotation);

      //SRAD' built in interval restriction isn't needed here. We need negative intervals.

      coord returncycle(std::pair<double,double>{0,0});

      double avg_angle = rel_orientation_change/2.0;

      if (rel_orientation_change == 0){
        returncycle.x = EncoderDistanceValues[2];
        returncycle.y = EncoderDistanceValues[1];
      } else {
        returncycle.y = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[1]/rel_orientation_change) +
        wheels[ENCODER_Position_RIGHT].Distance_CenterOfRotation));

        returncycle.x = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[2]/rel_orientation_change) +
        wheels[ENCODER_Position_BACK].Distance_CenterOfRotation));
      }

      returncycle = returncycle.transform_matrix(-(precycle.angle+avg_angle-(M_PI/2)));

      precycle += returncycle;
      precycle.angle.value -= rel_orientation_change;

      return precycle;
    }


    //Candidate revision of prexisting odom function. Main improvement is less compounding error from adding angles. To be verified
    Position cycleV2(Position precycle){
      std::array EncoderDistanceValues = wheels.get_distances_nonpointer(); //compute all distances, reset odometry wheels, append delta distance to total dist
      std::array EncoderDistanceTotalValues = wheels.get_distancesTotal_nonpointer(); //return array of total distances including new dist from get_distance_nonpointer

      //Assuming forwards is 0rad, CCW is positive we calculate the relative offset
      //All coords are prior to move fyi.
      SRAD raw_global_angle =
        (EncoderDistanceTotalValues[0]-EncoderDistanceTotalValues[1]) /
        (wheels[ENCODER_Position_LEFT].Distance_CenterOfRotation +
        wheels[ENCODER_Position_RIGHT].Distance_CenterOfRotation);

      double rel_orientation_change = SRAD().findDiff(raw_global_angle,precycle.angle);

      //SRAD' built in interval restriction isn't needed here. We need negative intervals.

      coord returncycle(std::pair<double,double>{0,0});

      double avg_angle = rel_orientation_change/2.0;

      if (rel_orientation_change == 0){
        returncycle.x = EncoderDistanceValues[2];
        returncycle.y = EncoderDistanceValues[1];
      } else {
        returncycle.y = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[1]/rel_orientation_change) +
        wheels[ENCODER_Position_RIGHT].Distance_CenterOfRotation));

        returncycle.x = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[2]/rel_orientation_change) +
        wheels[ENCODER_Position_BACK].Distance_CenterOfRotation));
      }

      returncycle = returncycle.transform_matrix(-(precycle.angle+avg_angle-(M_PI/2)));

      precycle += returncycle;
      precycle.angle = raw_global_angle;

      return precycle;
    }

    //IMU odometry function
    Position cycleIMU(Position precycle, SRAD new_heading){
      std::array EncoderDistanceValues = wheels.get_distances_nonpointer();
      //its a 50/50 that get_distances_nonpointer works

      //Assuming forwards is 0rad, CCW is positive we calculate the relative offset
      //All coords are prior to move fyi.
      double rel_orientation_change = new_heading.findDiff(new_heading,precycle.angle);

      //SRAD' built in interval restriction isn't needed here. We need negative intervals.

      coord returncycle(std::pair<double,double>{0,0});

      double avg_angle = rel_orientation_change/2.0000000;

      if (rel_orientation_change == 0){
        returncycle.x = EncoderDistanceValues[2];
        returncycle.y = EncoderDistanceValues[1];
      } else {
        returncycle.y = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[1]/rel_orientation_change) +
        wheels[ENCODER_Position_RIGHT].Distance_CenterOfRotation));

        returncycle.x = double(2.0*sin(avg_angle) *
        ((EncoderDistanceValues[2]/rel_orientation_change) +
        wheels[ENCODER_Position_BACK].Distance_CenterOfRotation));
      }

      returncycle = returncycle.transform_matrix(-(precycle.angle+avg_angle-(M_PI/2)));

      precycle += returncycle;
      precycle.angle.value = new_heading;

      return precycle;
    }
  };
}

#endif
