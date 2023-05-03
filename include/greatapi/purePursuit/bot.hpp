#ifndef BOT_PUREP_HPP
#define BOT_PUREP_HPP

#include "target.hpp"

namespace purePursuit {
    struct Bot {
        long double xPos = 0, yPos = 0;
        
        Target *target;
        
        int stage = 0;
        bool hnorth = true, heast = true;

        Bot(Target *target_) {
            target = target_;
        }

        std::pair<double, double> updatePosition(double x, double y) {
            xPos = x;
            yPos = y;

            return std::make_pair(target->xPos, target->yPos);
        }

        double btDist() {
            return sqrt(pow(xPos - target->xPos, 2) + pow(yPos - target->yPos, 2));
        }

    } ;
}



#endif