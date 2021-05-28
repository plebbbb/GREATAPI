//A header guard
#pragma once
#include "greatapi/basic_datatypes.hpp"
#include "main.h"

#ifndef TWHEEL_HPP
#define TWHEEL_HPP

//attempted odometry rewrite
namespace greatapi{

  //tracking wheel parent class
  struct TWheel{
    double WRadius;
    TWheel(double WR):WRadius(WR){};
    virtual double get_distance();
  };

  //old V4 ADI encoder tracking wheel
  struct TWheel_ADIEncoder : public TWheel{
    pros::ADIEncoder sensor;
    double WRadius;

    //normal brain mounted ADI port
    TWheel_ADIEncoder(int portA, int portB, bool direction, double WheelDiam):sensor(portA, portB, direction),TWheel(WheelDiam/double(2)){}

    //ADI expander port. First tuple term is the ADI expander's V5 port, other two are ports A and B.
    TWheel_ADIEncoder(std::tuple<int,int,int> port, bool direction, double WheelDiam):sensor(port,direction),TWheel(WheelDiam/double(2)){}

    //turns the degrees output of the ADIEncoder into radians, and the multiplying by radius to get the total spun distance(probably inches)
    double get_distance(){
      return DegToRad(double(sensor.get_value())) * WRadius;
    }
  };

  //new V5 rotation sensor tracking wheel
  struct TWheel_RotationSensor : public TWheel{
    pros::Rotation sensor;
    double SumRotation; //Sum, as new rotation sensor doesnt have a sum angle

    //v5 port constructor
    TWheel_RotationSensor(int port, bool direction, double WheelDiam):sensor(port),TWheel(WheelDiam/double(2)){
      sensor.set_reversed(direction); //reverse sensor depending on direction constructor parameter.
    }

    //turns the degrees output of the ADIEncoder into radians, and the multiplying by radius to get the total spun distance(probably inches)
    double get_distance(){
      double ang = sensor.get_angle();
      SumRotation += (ang > double(180)) ? ang-double(360) : ang; //if yielding bigger than 180, it is going backwards. We need a negative angle for that so we subtract 360.
      sensor.reset(); //reset rotation sensor
      return SumRotation * WRadius;
    }
  };

  //for the people insane enough to run odometry on V5 motor encoders
  struct TWheel_Motor : public TWheel {
    pros::Motor sensor;

    /*
      Pros gearsets have weird names. Below explains what they actually are:
        E_MOTOR_GEARSET_36 - 100rpm/Red cartridge
        E_MOTOR_GEARSET_18 - 200rpm/Green cartridge
        E_MOTOR_GEARSET_06 - 600rpm/Blue cartridge
    */
    TWheel_Motor(int port, pros::motor_gearset_e_t gearset, bool direction, double WheelDiam):sensor(port,gearset,direction,pros::E_MOTOR_ENCODER_DEGREES),TWheel(WheelDiam/double(2)){}

    double get_distance(){
      return DegToRad(double(sensor.get_position())) * WRadius;
    }

  };

}
#endif
