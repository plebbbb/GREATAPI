

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

        struct bot {
            odometry::odometry* odom;
            position curPos = position(coord(0, 0), SRAD(0));
            position targetPos = position(coord(0, 0), SRAD(0));

            purePursuit::Target target;

            bool moveDrive = true;
            double voltageCap = 12000;

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
            }

            void pos_control();
        } ;

        struct bot_tank : public bot {
            pros::Motor_Group * leftMotors;
            pros::Motor_Group * rightMotors;

            bool translating = false;
            bool reverseDrive = false;

            double kPAngle = 18000;
            double kIAngle = 7000;

            controlelement *PY = new greatapi::Proportional(1200, std::pair(__INT_MAX__, -__INT_MAX__));          
            controlelement *IY = new greatapi::Integral(0, std::pair(3000, -3000));                              
            controlelement *DY = new greatapi::Derivative(1800, std::pair(__INT_MAX__, -__INT_MAX__));            
            std::vector<greatapi::controlelement *> PIDYElements = {PY, IY, DY};
            control_loop PIDY = control_loop(PIDYElements, std::pair(12000, -12000));


            controlelement *PAngle = new greatapi::Proportional(kPAngle, std::pair(__INT_MAX__, -__INT_MAX__));     
            controlelement *IAngle = new greatapi::Integral(kIAngle, std::pair(2000, -2000));                        
            controlelement *DAngle = new greatapi::Derivative(320000, std::pair(__INT_MAX__, -__INT_MAX__));
            std::vector<greatapi::controlelement *> PIDAngleElements = {PAngle, IAngle, DAngle};
            control_loop PIDAngle = control_loop(PIDAngleElements, std::pair(12000, -12000));

            bot_tank(odometry::odometry* odom_, pros::Motor_Group * leftMotors, pros::Motor_Group * rightMotors) : bot(odom_) {
                this->leftMotors = leftMotors;
                this->rightMotors = rightMotors;
            }

            void pos_control() {
                targetPos = curPos;

                while (true) {
                    total_error = sqrt(pow(targetPos.x - curPos.x, 2) + pow(targetPos.y - curPos.y, 2));

                    error = greatapi::coord(curPos, targetPos);
                    error.self_transform_matrix(greatapi::SRAD(-1.0 * curPos.angle));

                    // if (translating && fabs((double) error.x) > 0.5) {
                    if (translating && total_error > 2) {
                        if (reverseDrive) targetPos.angle = greatapi::SRAD(atan2(targetPos.y - curPos.y, targetPos.x - curPos.x) + PI / 2);
                        else targetPos.angle = greatapi::SRAD(atan2(targetPos.y - curPos.y, targetPos.x - curPos.x) - PI / 2);
                    }
                    
                    double yMove = PIDY.update(error.y, 0);
                    double anglePow = -PIDAngle.update(greatapi::findDiff(curPos.angle, targetPos.angle), 0);

                    if (yMove > voltageCap) {
                        yMove = voltageCap;
                    } else if (yMove < -voltageCap) {
                        yMove = -voltageCap;
                    }

                    if (anglePow > voltageCap) {
                        anglePow = voltageCap;
                    } else if (anglePow < -voltageCap) {
                        anglePow = -voltageCap;
                    }
                    

                    double lPower = (yMove + anglePow);
                    double rPower = (yMove - anglePow);

                    if (moveDrive) {
                        leftMotors->move_voltage(lPower);
                        rightMotors->move_voltage(rPower);
                    }
                    
                    // pros::screen::print(TEXT_SMALL, 4, "Angle target: %.2f X: %.2f Y: %.2f\n", targetPos.angle / PI * 180, targetPos.x, targetPos.y);
                    // pros::screen::print(TEXT_SMALL, 5, "Total error: %.2f\n", total_error);

                    // printf("Angle target: %.2f X: %.2f Y: %.2f\n", targetPos.angle / PI * 180, (double) targetPos.x, (double) targetPos.y);
                    // printf("Total error: %.2f\n", total_error);

                    pros::delay(5);
                }
            }

        } ;


        struct bot_xdrive {
            pros::Motor * lfMotor;
            pros::Motor * rfMotor;
            pros::Motor * rbMotor;
            pros::Motor * lbMotor;

            controlelement *PY = new greatapi::Proportional(1200, std::pair(__INT_MAX__, -__INT_MAX__));          
            controlelement *IY = new greatapi::Integral(0, std::pair(3000, -3000));                              
            controlelement *DY = new greatapi::Derivative(1800, std::pair(__INT_MAX__, -__INT_MAX__));            
            std::vector<greatapi::controlelement *> PIDYElements = {PY, IY, DY};
            control_loop PIDY = control_loop(PIDYElements, std::pair(12000, -12000));

            controlelement *PX = new greatapi::Proportional(1200, std::pair(__INT_MAX__, -__INT_MAX__));          
            controlelement *IX = new greatapi::Integral(0, std::pair(3000, -3000));                              
            controlelement *DX = new greatapi::Derivative(1800, std::pair(__INT_MAX__, -__INT_MAX__));            
            std::vector<greatapi::controlelement *> PIDXElements = {PX, IX, DX};
            control_loop PIDX = control_loop(PIDXElements, std::pair(12000, -12000));

            controlelement *PAngle = new greatapi::Proportional(17000, std::pair(__INT_MAX__, -__INT_MAX__));     
            controlelement *IAngle = new greatapi::Integral(5000, std::pair(2000, -2000));                        
            controlelement *DAngle = new greatapi::Derivative(320000, std::pair(__INT_MAX__, -__INT_MAX__));
            std::vector<greatapi::controlelement *> PIDAngleElements = {PAngle, IAngle, DAngle};
            control_loop PIDAngle = control_loop(PIDAngleElements, std::pair(12000, -12000));

            bot_xdrive(odometry::odometry* odom_, pros::Motor lfm, pros::Motor rfm, pros::Motor rbm, pros::Motor lbm) {
                
            }

        } ;
    }
}