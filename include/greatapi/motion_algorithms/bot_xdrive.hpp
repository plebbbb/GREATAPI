#include "bot.hpp"

namespace greatapi {
    namespace motion {
        struct bot_xdrive : bot {
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

            /**
             * @brief Construct a new bot xdrive object. Pass the pointer to the greatapi odom object, followed by 
             * the motors in this sequence: Left Front, Right Front, Right Rear, Left Rear. 
             * 
             * @param odom_ 
             * @param lfm 
             * @param rfm 
             * @param rbm 
             * @param lbm 
             */

            bot_xdrive(odometry::odometry* odom_, pros::Motor* lfm, pros::Motor* rfm, pros::Motor* rbm, pros::Motor* lbm) : bot(odom_) {
                lfMotor = lfm;
                rfMotor = rfm;
                rbMotor = rbm;
                lbMotor = lbm;
            }

            void pos_control() {
                targetPos = curPos;

                while (true) {
                    total_error = sqrt(pow(targetPos.x - curPos.x, 2) + pow(targetPos.y - curPos.y, 2));

                    greatapi::coord error(curPos, targetPos);
                    // pros::screen::print(pros::E_TEXT_SMALL, 3, "X error: %.2f  Y error: %.2f", error.x, error.y);
                    error.self_transform_matrix(greatapi::SRAD(-1.0 * curPos.angle));

                    double xMove = PIDX.update(error.x, 0);
                    double yMove = PIDY.update(error.y, 0);
                    double anglePow = PIDAngle.update(greatapi::findDiff(curPos.angle, targetPos.angle), 0);

                    if (yMove > voltageCap) {
                        yMove = voltageCap;
                    } else if (yMove < -voltageCap) {
                        yMove = -voltageCap;
                    }
                    
                    if (xMove > voltageCap) {
                        xMove = voltageCap;
                    } else if (xMove < -voltageCap) {
                        xMove = -voltageCap;
                    }

                    double lfPower = yMove + xMove - anglePow;
                    double rfPower = yMove - xMove + anglePow;
                    double lbPower = yMove - xMove - anglePow;
                    double rbPower = yMove + xMove + anglePow;

                    if (lfPower > voltageCap) {
                        lfPower = voltageCap;
                    } else if (lfPower < -voltageCap) {
                        lfPower = -voltageCap;
                    }

                    if (rfPower > voltageCap) {
                        rfPower = voltageCap;
                    } else if (rfPower < -voltageCap) {
                        rfPower = -voltageCap;
                    }

                    if (rbPower > voltageCap) {
                        rbPower = voltageCap;
                    } else if (rbPower < -voltageCap) {
                        rbPower = -voltageCap;
                    }

                    if (lbPower > voltageCap) {
                        lbPower = voltageCap;
                    } else if (lbPower < -voltageCap) {
                        lbPower = -voltageCap;
                    }


                    if (fabs(lfPower - anglePow) > voltageCap) {
                        int subt = signum(lfPower - anglePow) * voltageCap - lfPower + anglePow;
                        lfPower -= subt;
                        rbPower -= subt;

                    }

                    if (moveDrive) {
                        lfMotor->move_voltage(lfPower);
                        rfMotor->move_voltage(rfPower);
                        lbMotor->move_voltage(lbPower);
                        rbMotor->move_voltage(rbPower);
                    }
                }
            }

        } ;
    }
}