#include "main.h"
#include "greatapi/greatapi.hpp"
#include "pros/motors.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

//------------------------------------------------------------------------------
// Port configuration
#define l1_motor_port 1 //1 is top, 3 is bottom
#define l2_motor_port 2
#define l3_motor_port 3
#define r1_motor_port 4
#define r2_motor_port 5
#define r3_motor_port 6

#define intake_port 10
#define flywheel_port_1 8
#define imu_port 11
#define angler1_port 'A'
#define expansion_port 'B'
#define discFullSensor_port 'C'
#define shootSensor_port 'F'
#define tripleIndexerPort 'D'
#define singleIndexerPort 'E'
// Tracking wheel ports
#define left_tracking_port 13
#define rear_tracking_port 12
// Odometry constants
#define WHEEL_DIST_LR 5.6875
#define XWHEEL_DIST_CENTER 1.25
#define wheelDiam 3.2

#define powerFactor 1.5

//------------------------------------------------------------------------------
// Global PROS objects and variables
// Motors
inline pros::Motor r1_motor = pros::Motor(l1_motor_port, MOTOR_GEARSET_06, false,
                            MOTOR_ENCODER_DEGREES);
inline pros::Motor r2_motor = pros::Motor(l2_motor_port, MOTOR_GEARSET_06, true,
                            MOTOR_ENCODER_DEGREES);
inline pros::Motor r3_motor = pros::Motor(l3_motor_port, MOTOR_GEARSET_06, false,
                            MOTOR_ENCODER_DEGREES);
inline pros::Motor l1_motor = pros::Motor(r1_motor_port, MOTOR_GEARSET_06, true,
                            MOTOR_ENCODER_DEGREES);
inline pros::Motor l2_motor = pros::Motor(r2_motor_port, MOTOR_GEARSET_06, false,
                            MOTOR_ENCODER_DEGREES);
inline pros::Motor l3_motor = pros::Motor(r3_motor_port, MOTOR_GEARSET_06, true,
                            MOTOR_ENCODER_DEGREES);

inline pros::Motor_Group left_drive({l1_motor, l2_motor, l3_motor});
inline pros::Motor_Group right_drive({r1_motor, r2_motor, r3_motor});

inline pros::Motor intake(intake_port, MOTOR_GEARSET_06, true,
                          MOTOR_ENCODER_DEGREES);
inline pros::Motor flywheel(flywheel_port_1, MOTOR_GEARSET_06, false,
                              MOTOR_ENCODER_DEGREES);

// Sensors
inline pros::ADIAnalogIn discFullSensor(discFullSensor_port);
inline pros::ADIAnalogIn shootSensor(shootSensor_port);
// controller
inline pros::Controller master(CONTROLLER_MASTER);
// pneumatics
inline pros::ADIDigitalOut angler1Piston(angler1_port, 0);
inline pros::ADIDigitalOut expansionPiston(expansion_port, 0);
inline pros::ADIDigitalOut tripleIndexerPiston(tripleIndexerPort, 0);
inline pros::ADIDigitalOut singleIndexerPiston(singleIndexerPort, 0);

//------------------------------------------------------------------------------
// GREATAPI objects and variables
// sensors
inline greatapi::TWheel_RotationSensor left_encoder = 
    greatapi::TWheel_RotationSensor(left_tracking_port, true, wheelDiam);
inline greatapi::TWheel_RotationSensor rear_encoder = 
    greatapi::TWheel_RotationSensor(rear_tracking_port, true, wheelDiam);
// odometry object
inline greatapi::odometry::IMU_odom_rotation imuRotation
    (imu_port, -1.01);

inline greatapi::odometry::odometry odom = greatapi::odometry::odometry(&rear_encoder, greatapi::inches(XWHEEL_DIST_CENTER), &left_encoder,
         greatapi::inches(- WHEEL_DIST_LR / 2.0), &imuRotation);

inline greatapi::motion::bot_tank robot(&odom, &left_drive, &right_drive);

// total position error
//voltage cap to reduce speed

//------------------------------------------------------------------------------
// other variables

inline bool autonomousState = false;
inline bool red_team = true;

#endif // CONFIG_HPP
