
#include "helpers.hpp"
#include "odometry/odometry.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "units/angle_units/srad.hpp"
#include "units/coordinate/coord.hpp"
#include "units/coordinate/position.hpp"
#include "purePursuit/purePursuit.hpp"
#include "control_loops/control_loops.hpp"
#include "main.h"

namespace greatapi {
    namespace motion {
        #define PI 3.1415926535897932384626433832795

        #define rotateVoltCap 12000
        #define moveRotVoltCap 6000
        #define moveVoltCap 10000

        struct bot {
            odometry::odometry* odom;
            position curPos = position(coord(0, 0), SRAD(0));
            position targetPos = position(coord(0, 0), SRAD(0));

            purePursuit::Target target;

            bool moveDrive = true;
            double voltageCap = moveVoltCap;
            double rotVCap = moveRotVoltCap;

            bot(odometry::odometry* odom_) {
                odom = odom_;
            }

            double total_error = 0;
            coord error = coord(0, 0);

            void odomLooper() {
                while (true) {
                    curPos = odom->calculateposition(curPos);
                    pros::delay(5);
                }
            }

            double btDist() {
                return sqrt(pow(curPos.x.value - target.xPos, 2) + pow(curPos.y.value - target.yPos, 2));
            }

            void calibrate() {
                odom->rotationcalc->calibrate();
                odom->tare();
            }

            void pos_control();

            

            void rotate(double angleDeg, double errorStop);



            /**
             * @brief Initializes the Bot object for use. Takes the starting orientation of the bot in RADIANS, CW is positive.
             * 
             * @param startOrientation 
             */
            void init(SRAD startOrientation) {
                odom->tare();

                targetPos.angle = startOrientation;
                curPos.angle = startOrientation;
                odom->rotationcalc->DOffset = -startOrientation;

                pros::Task odomTask = pros::Task(&bot::odomLooper);
                pros::Task posControlTask = pros::Task(&bot::pos_control);
            }

            

        } ;

        


        
    }
}