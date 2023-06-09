#ifndef BOT_HPP
#define BOT_HPP 

#include "greatapi/helpers.hpp"
#include "greatapi/odometry/odometry.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/purePursuit/purePursuit.hpp"
#include "greatapi/control_loops/control_loops.hpp"
#include "main.h"

namespace greatapi {
    namespace motion {

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

            double total_error = 0;
            coord error = coord(0, 0);

            bot(odometry::odometry* odom_) {
                odom = odom_;
            }

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

            virtual void pos_control() = 0;

            virtual void rotate(double angleDeg, double errorStop) = 0;



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

                pros::Task odomTask ([=] {
                    odomLooper();
                }, "odom_task");

                pros::Task motionTask ([=] {
                    pos_control();
                }, "motion_task");
            }
        } ;
    }
}
#endif