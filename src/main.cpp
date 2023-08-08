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

	// robot.rotate(90, 0);

	// robot.translate(0, 30, false, true, 0);
	// robot.translate(0, 0, true, false, 0);

	// robot.translatevl(10, 30, false, 4000, true, 0);

	// robot.rtranslatevl(0, 10, false, 4000, true, 0);
	// robot.rtranslatevl(0, -10, true, 4000, true, 0);
	

	// robot.translatevl(0, 0, true, 4000, true, 0);



    std::pair<double, double> path1[] = {std::make_pair(0, 15), std::make_pair(15, 15), std::make_pair(15, 30)};
	robot.ptranslatevl(path1, 3, false, 8000, false, 0);
	pros::delay(5000);

	
}


void opcontrol() {
	autonomous();
	
	while(true){
		
		pros::delay(10);
	}
}
