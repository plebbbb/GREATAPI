#ifndef BOT_HPP
#define BOT_HPP 

#include "greatapi/helpers.hpp"
#include "greatapi/odometry/odometry.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/purePursuit/purePursuit.hpp"
#include "greatapi/control_loops/control_loops.hpp"
#include "config.hpp"

namespace greatapi {
    namespace motion {

        struct bot {
            odometry::odometry* odom;
            position curPos = position(coord(0, 0), SRAD(0));
            position targetPos = position(coord(0, 0), SRAD(0));

            purePursuit::Target target = purePursuit::Target();

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

            /**
             * @brief Calibrates all sensors needed for odometry/position control. 
             * Call this function in initalize(). If IMU odom is used, this will block execution for 2 seconds.
             * 
             */
            void calibrate() {
                odom->rotationcalc->calibrate();
                odom->tare();
            }

            virtual void pos_control() = 0;

            virtual void rotate(double angleDeg, double errorStop) = 0;



            /**
             * @brief Initializes the Bot object for use. Takes the starting orientation of the bot in RADIANS, CW is positive.
             * Run this part at the start of your autonomous routine.
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
                pros::delay(50);
            }
        } ;
    }
}
#endif