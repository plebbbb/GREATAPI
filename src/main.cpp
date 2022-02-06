#include "main.h"
#include "greatapi/greatapi.hpp"


void initialize() {
}


void disabled() {}


void competition_initialize() {}

void autonomous() {}


void opcontrol() {

		//TWheel is an abstract class. You should be using the constructor of specific TWheels
	greatapi::TWheel* leftwheel = new greatapi::TWheel_Motor(5, pros::E_MOTOR_GEARSET_18,true, 2.75); //V5 motor, 200RPM, reversed, 2.75in wheel

	//For information on TWheel, please see the TWheel section of the site.
	greatapi::TWheel* rightwheel = new greatapi::TWheel_RotationSensor(4, false, 4); //V5 rotation sensor, not reversed, 4in wheel
	//note that we are making TWheel* (TWheel pointers), not TWheels.

	greatapi::TWheel* rearwheel = new greatapi::TWheel_ADIEncoder('A','B',false,2.75); //V4 rotation encoder

	greatapi::odometry::TWheel_odom_rotation example = *new greatapi::odometry::TWheel_odom_rotation(leftwheel,rightwheel,15); //15 inches between

	greatapi::odometry::odometry test(leftwheel, greatapi::inches(15), rearwheel, greatapi::inches(15), &example);

	greatapi::position location(greatapi::coord(),greatapi::SRAD(0));

	while(true){
		test.calculateposition(location);
		pros::delay(10);
	}
}
