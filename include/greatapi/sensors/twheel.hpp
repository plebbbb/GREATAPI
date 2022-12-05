//A header guard
#pragma once
#include "greatapi/basic_datatypes.hpp"
#include "main.h"

#ifndef TWHEEL_HPP
#define TWHEEL_HPP

//attempted odometry rewrite
namespace greatapi{

  //tracking wheel parent class
  struct TWheel {
    distance WRadius;
    TWheel(distance WR):WRadius(WR){};
    virtual distance get_distance() = 0;
    virtual void reset() = 0;
  };

  //old V4 ADI encoder tracking wheel
  struct TWheel_ADIEncoder : public TWheel{
    pros::ADIEncoder sensor;

    //normal brain mounted ADI port
    TWheel_ADIEncoder(int portA, int portB, bool direction, distance WheelDiam):sensor(portA, portB, direction),TWheel(WheelDiam.value/double(2)){}

    //ADI expander port. First tuple term is the ADI expander's V5 port, other two are ports A and B.
    TWheel_ADIEncoder(std::tuple<int,int,int> port, bool direction, distance WheelDiam):sensor(port,direction),TWheel(WheelDiam.value/double(2)){}

    //turns the degrees output of the ADIEncoder into radians, and the multiplying by radius to get the total spun distance(probably inches)
    distance get_distance(){
      return (double)angle(degrees(sensor.get_value())) * (double)WRadius; //the degrees class is converted into an angle class(which is in radians)
    }

    //resets the ADIEncoder
    void reset(){
      sensor.reset();
    }
  };

  //new V5 rotation sensor tracking wheel
  struct TWheel_RotationSensor : public TWheel{
    pros::Rotation sensor;

    //v5 port constructor
    TWheel_RotationSensor(int port, bool direction, double WheelDiam):sensor(port),TWheel(WheelDiam/double(2)){
      sensor.set_data_rate(5);
      sensor.reset_position();
      sensor.set_reversed(direction); //reverse sensor depending on direction constructor parameter.
    }

    //turns the degrees output of the ADIEncoder into radians, and the multiplying by radius to get the total spun distance(probably inches)
    distance get_distance(){
      return (double)angle(degrees((double)sensor.get_position() / 100.0)) * (double)WRadius; //the degrees class is converted into an angle class(which is in radians)
    }

    //resets the Rotation Sensor
    void reset(){
      sensor.reset_position();
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

    distance get_distance(){
      return (double)angle(degrees(sensor.get_position())) * (double)WRadius;
    }

    //resets the motor encoder
    void reset(){
      sensor.tare_position();
    }
  };

}
#endif
