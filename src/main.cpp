#include "main.h"
#include "greatapi/greatapi.hpp"
#include "globals.hpp"


void initialize() {
	robot.calibrate();
}


void disabled() {}


void competition_initialize() {}

void autonomous() {
	robot.init(greatapi::SRAD(0));
}


void opcontrol() {

	
	while(true){
		
		pros::delay(10);
	}
}
