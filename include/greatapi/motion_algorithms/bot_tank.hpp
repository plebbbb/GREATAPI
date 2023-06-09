#include "bot.hpp"
#include <utility>

namespace greatapi {
    namespace motion {
        struct bot_tank : bot {
            pros::Motor_Group * leftMotors;
            pros::Motor_Group * rightMotors;

            bool overRideHeading = true;
            bool reverseDrive = false;

            double kPAngle = 18000;
            double kIAngle = 5000;

            double kPRotate = 30000;

            controlelement *PY = new greatapi::Proportional(1200, std::pair(__INT_MAX__, -__INT_MAX__));          
            controlelement *IY = new greatapi::Integral(0, std::pair(3000, -3000));                              
            controlelement *DY = new greatapi::Derivative(1800, std::pair(__INT_MAX__, -__INT_MAX__));            
            std::vector<greatapi::controlelement *> PIDYElements = {PY, IY, DY};
            control_loop PIDY = control_loop(PIDYElements, std::pair(12000, -12000));


            controlelement *PAngle = new greatapi::Proportional(kPAngle, std::pair(__INT_MAX__, -__INT_MAX__));     
            controlelement *IAngle = new greatapi::Integral(kIAngle, std::pair(2000, -2000));                        
            controlelement *DAngle = new greatapi::Derivative(160000, std::pair(__INT_MAX__, -__INT_MAX__));
            std::vector<greatapi::controlelement *> PIDAngleElements = {PAngle, IAngle, DAngle};
            control_loop PIDAngle = control_loop(PIDAngleElements, std::pair(12000, -12000));

            /**
             * @brief Construct a new bot tank object
             * 
             * @param odom_ 
             * @param leftMotors 
             * @param rightMotors 
             */
            bot_tank(odometry::odometry* odom_, pros::Motor_Group * leftMotors, pros::Motor_Group * rightMotors) : bot(odom_) {
                this->leftMotors = leftMotors;
                this->rightMotors = rightMotors;
            }

            /**
             * @brief function that runs in a separate thread to control the bot's position
             * 
             */
            void pos_control() {
                targetPos = curPos;

                while (true) {
                    total_error = sqrt(pow(targetPos.x - curPos.x, 2) + pow(targetPos.y - curPos.y, 2));

                    error = greatapi::coord(curPos, targetPos);
                    error.self_transform_matrix(greatapi::SRAD(-1.0 * curPos.angle));

                    // if (translating && fabs((double) error.x) > 0.5) {
                    if (!overRideHeading && total_error > 2) {
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

                    if (anglePow > rotVCap) {
                        anglePow = rotVCap;
                    } else if (anglePow < -rotVCap) {
                        anglePow = -rotVCap;
                    }

                    if (yMove + fabs(anglePow) > voltageCap) {
                        yMove = voltageCap - fabs(anglePow);
                    } else if (yMove - fabs(anglePow) < -voltageCap) {
                        yMove = -voltageCap + fabs(anglePow);
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

                    pros::delay(10);
                }
            }

            /**
            * rotates the bot to the specified absolute heading. Blocks execution until the bot is at the specified heading.
            * 
            * \param angle the absolute heading to rotate to. Clockwise is positive, counter-clockwise is negative
            * \param errorStop DEGREES the function will stop the bot if the error is greater than the error threshold. IF 0, default is 2 degrees
            */
            void rotate(double angleDeg, double errorStop) {

                PAngle->factor = kPRotate;
                // IAngle->factor = 0;
                overRideHeading = false;
                rotVCap = rotateVoltCap;
                voltageCap = fmin(rotateVoltCap * 1.5, 12000);

                greatapi::SRAD angle = greatapi::SRAD((-1.0 * angleDeg) * PI / 180.0);
                targetPos.angle = angle;
                errorStop = errorStop == 0 ? 1.5 : errorStop;

                int stuckTimer = 0;
                double prevError = fabs(greatapi::findDiff(curPos.angle, targetPos.angle));

                while (fabs(greatapi::findDiff(curPos.angle, targetPos.angle)) > greatapi::degrees(errorStop) 
                        && stuckTimer < 50) {
                    if (fabs(greatapi::findDiff(curPos.angle, targetPos.angle)) - prevError < 1.0 / 180 * PI) {
                        stuckTimer++;
                    } else {
                        stuckTimer = 0;
                    }
                    prevError = fabs(greatapi::findDiff(curPos.angle, targetPos.angle));
                    pros::delay(50);
                }

                voltageCap = moveVoltCap;
                rotVCap = moveRotVoltCap;

                PAngle->factor = kPAngle;
                IAngle = new greatapi::Integral(kIAngle, std::pair(1000, -1000));           
                return;
            }

            /**
            * translates the robot to absolute coordinates. DOES NOT BLOCK EXECUTION
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param maxVoltage the maximum voltage to send to the motors
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target.
            */
            void translatevl(double x, double y, bool revDrive, double maxVoltage, bool goHeading, bool reverseHeading) {
                if (goHeading) {
                    if (reverseHeading) {
                        rotate(90 - (atan2(y - curPos.y, x - curPos.x) + PI) / PI * 180.0, 0);
                    } else {
                        rotate(90 - (atan2(y - curPos.y, x - curPos.x)) / PI * 180.0, 0);
                    }
                }
                targetPos.x = x;
                targetPos.y = y;
                reverseDrive = revDrive;

                voltageCap = maxVoltage;
                rotVCap = rotateVoltCap;

                total_error = sqrt(pow(targetPos.x - curPos.x, 2) + pow(targetPos.y - curPos.y, 2));
                overRideHeading = true;
                if (reverseDrive) targetPos.angle = greatapi::SRAD(atan2(targetPos.y - curPos.y, targetPos.x - curPos.x) + PI / 2);
                else targetPos.angle = greatapi::SRAD(atan2(targetPos.y - curPos.y, targetPos.x - curPos.x) - PI / 2);

                return;
            }

            /**
            * translates the robot to absolute coordinates. DOES NOT BLOCK EXECUTION
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target.
            */
            void translate(double x, double y, bool revDrive, bool goHeading, bool reverseHeading) {
                translatevl(x, y, revDrive, moveVoltCap, goHeading, reverseHeading);
                return;
            }

            /**
            * translates the robot to absolute coordinates. Blocks execution. 
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param maxVoltage the maximum voltage to send to the motors
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target
            * \param distToStopBlock the distance from target to stop blocking the function. IF 0, it will default to 0.8
            */
            void translatevl(double x, double y, bool revDrive, double maxVoltage, bool goHeading, bool reverseHeading, double distToStopBlock) {
                translatevl(x, y, revDrive, maxVoltage, goHeading, reverseHeading);
                if (distToStopBlock == 0) distToStopBlock = 1;
                int stuckTimer = 0;
                double prevError = total_error;
                pros::delay(20);
                while (total_error > distToStopBlock && stuckTimer < 50) { // Keep looping until at target, abort to next movement if stuck for 1.5 seconds
                    if (fabs(total_error - prevError) < 0.06) {
                        stuckTimer++;
                    } else {
                        stuckTimer = 0;
                    }
                    prevError = total_error;
                    pros::delay(20);

                }
                return;
            }

            /**
            * translates the robot to absolute coordinates. Blocks execution. 
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target
            * \param distToStopBlock the distance from target to stop blocking the function. IF 0, it will default to 0.8
            */
            void translate(double x, double y, bool revDrive, bool goHeading, bool reverseHeading, double distToStopBlock) {
                translatevl(x, y, revDrive, moveVoltCap, goHeading, reverseHeading, distToStopBlock);
                return;
            }

            /**
            * translates the robot to relative coordinates. DOES NOT BLOCK EXECUTION
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param maxVoltage the maximum voltage to send to the motors
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target.
            */
            void rtranslatevl(double x, double y, bool revDrive, double maxVoltage, bool goHeading, bool reverseHeading) {
                translatevl(((double) targetPos.x) + x, ((double) targetPos.y) + y, revDrive, maxVoltage, goHeading, reverseHeading);
            }
            /**
            * translates the robot to relative coordinates. DOES NOT BLOCK EXECUTION
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target.
            */
            void rtranslate(double x, double y, bool revDrive, bool goHeading, bool reverseHeading) {
                translate(((double) targetPos.x) + x, ((double) targetPos.y) + y, revDrive, goHeading, reverseHeading);
            }
            /**
            * translates the robot to relative coordinates. Blocks execution. 
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param maxVoltage the maximum voltage to send to the motors
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target
            * \param distToStopBlock the distance from target to stop blocking the function. IF 0, it will default to 0.8
            */
            void rtranslatevl(double x, double y, bool revDrive, double maxVoltage, bool goHeading, bool reverseHeading, double distToStopBlock) {
                translatevl(((double) targetPos.x) + x, ((double) targetPos.y) + y, revDrive, maxVoltage, goHeading, reverseHeading, distToStopBlock);
            }
            /**
            * translates the robot to relative coordinates. Blocks execution. 
            * 
            * \param x the x coordinate to translate to
            * \param y the y coordinate to translate to
            * \param revDrive whether or not to reverse the drive direction
            * \param goHeading whether or not to point towards the target
            * \param reverseHeading whether or not to invert the heading when pointing towards the target
            * \param distToStopBlock the distance from target to stop blocking the function. IF 0, it will default to 0.8
            */
            void rtranslate(double x, double y, bool revDrive, bool goHeading, bool reverseHeading, double distToStopBlock) {
                translate(((double) targetPos.x) + x, ((double) targetPos.y) + y, revDrive, goHeading, reverseHeading, distToStopBlock);
            }

            void rtranslateDist(double dist, bool revDrive, double maxVoltage) {
                double x = (dist * cos(targetPos.angle));
                double y = (dist * sin(targetPos.angle));
                if (revDrive) {
                    x = -x;
                    y = -y;
                }
                rtranslatevl(x, y, revDrive, maxVoltage, false, false);
            }
            void rtranslateDist(double dist, bool revDrive) {
                rtranslateDist(dist, revDrive, moveVoltCap);
            }

            void ptranslatevl(std::pair<double, double> coords[], int pathLen, bool revDrive, double maxVoltage, bool goHeading, bool reverseHeading, double distToStopBlock) {    
    
                if (goHeading) {
                    if (reverseHeading) {
                        rotate(90 - (atan2(coords[0].second - curPos.y, coords[0].first - curPos.x) + PI) / PI * 180.0, 0);
                    } else {
                        rotate(90 - (atan2(coords[0].second - curPos.y, coords[0].first - curPos.x)) / PI * 180.0, 0);
                    }
                }
                purePursuit::Node nodes[pathLen + 1];

                nodes[0].xPos = (double) targetPos.x;
                nodes[0].yPos = (double) targetPos.y;

                for (int i = 1; i <= pathLen; i++) {
                    nodes[i].xPos = coords[i - 1].first;
                    nodes[i].yPos = coords[i - 1].second;
                }

                std::pair<purePursuit::Node, purePursuit::Node> path[pathLen];
                for (int i = 0; i < pathLen; i++) {
                    path[i].first = nodes[i];
                    path[i].second = nodes[i + 1];
                }

                reverseDrive = revDrive;
                voltageCap = maxVoltage;
                rotVCap = rotateVoltCap;
                
                overRideHeading = false;
                
                if (distToStopBlock == 0) distToStopBlock = 1;

                std::pair<double, double> targetPair;
                target.newPath(path, pathLen);

                int stuckTimer = 0;
                double prevError = total_error;
                pros::delay(20);

                greatapi::SRAD pathAngle = greatapi::SRAD(0);

                bool lastIn = false;


                //IAngle->factor = 0;


                // initialize the ksi to 5% as a parameter - then we can fine tune it 
                // line 40 - syntax ** for conditional??

                while (stuckTimer < 75) { // Keep looping until at target, abort to next movement if stuck for 1.5 seconds
                    if (fabs(total_error - prevError) < 0.08) {
                        stuckTimer++;
                    } else {
                        stuckTimer = 0;
                    }
                    prevError = total_error;

                    // pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Stage: %d  targetPos X: %.2f  Y: %.2f\n", target.stage, targetPair.first, targetPair.second);
                    // pros::screen::print(pros::E_TEXT_MEDIUM, 8, "xTrans: %.2f yTrans: %.2f\n", target.xTrans, target.yTrans);
                    // pros::screen::print(pros::E_TEXT_MEDIUM, 9, "xoy: %.2f\n", target.xoy);
                    // printf("stage: %d xPos: %.2f yPos: %.2f\nxTrans: %.2f yTrans: %.2f\nxoy: %.2f\n", target.stage, target.xPos, target.yPos, target.xTrans, target.yTrans, target.xoy);


                    target.updatePosition();
                    if (btDist() > visionRadius) {
                        target.bind(btDist());
                    }
                    targetPair = std::make_pair(target.xPos, target.yPos);

                    // if (!lastIn && fabs(greatapi::findDiff(curPos.angle, targetPos.angle)) < greatapi::degrees(18)) {
                    //     printf("T %.2f\n", (double) curPos.angle * 180 / PI);
                    //     IAngle = new greatapi::Integral(kIAngle, std::pair(1000, -1000));
                    //     lastIn = true;
                    // }

                    targetPos.x = targetPair.first;
                    targetPos.y = targetPair.second;

                    pros::delay(50);

                    if (target.atPathEnd) {
                        if (total_error < distToStopBlock) break;
                    }

                IAngle = new greatapi::Integral(kIAngle, std::pair(1000, -1000));
                
                }

                return;

            }

            void ptranslate(std::pair<double, double>*coords[], int pathLen, bool revDrive, bool goHeading, bool reverseHeading, double distToStopBlock) {
                ptranslatevl(*coords, pathLen, revDrive, moveVoltCap, goHeading, reverseHeading, distToStopBlock);
            }

        } ;
    }
}