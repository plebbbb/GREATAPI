#include <cmath>
#include <utility>
#include <vector>
#include "node.hpp"

#ifndef TARGET_HPP2
#define TARGET_HPP2

#define visionRadiusM 15
#define velocityM 10

namespace purePursuit {

    struct Target2 {

        long double xPos = 0;
        long double yPos = 0;

        long double botX = 0;
        long double botY = 0;
        int visionRadius = visionRadiusM;
        Node endpoint = Node(0, 0);
        
        std::pair<Node, Node> path[128];
        int pathLength = 0;

        int stage = 0, start = 0;

        //path vars
        long double m = 0, b = 0;
        double endX, endY;

        bool unlocked = false;

        bool atPathEnd = false;

        bool firstLoop = false;

        Target2() {}

        void computePathEq() {
            endX = path[stage].second.xPos;
            endY = path[stage].second.yPos;

            double startX = path[stage].first.xPos;
            double startY = path[stage].first.yPos;

            m = (endY - startY) / (endX - startX);

            b = startY - m * startX;
        }

        void newPath(std::pair<Node, Node> path_[], int pathLength_) {
            firstLoop = true;
            unlocked = true;

            //endpoint = Node(xh, yh);
            pathLength = pathLength_;
            
            for (int i = 0; i < 128; i++) {
                if (i < pathLength) {
                    path[i] = path_[i];
                }
            }

            xPos = path[0].first.xPos;
            yPos = path[0].first.yPos;

            computePathEq();
        }

        void updatePosition(double botX_, double botY_) {
            botX = botX_;
            botY = botY_;
            if (unlocked) {
                setStage();
                
                double a = m * m + 1;
                double b = 2 * m * (m * botX + b - botY);
                double c = pow(m * botX + b - botY, 2) - visionRadius * visionRadius;

                double discriminant = b * b - 4 * a * c;

                if (discriminant > 0) {
                    double solX1 = (-b + sqrt(discriminant))/ (2 * a);
                    double solX2 = (-b - sqrt(discriminant))/ (2 * a);

                    double solY1 = sqrt(visionRadius * visionRadius - solX1 * solX1);
                    double solY2 = sqrt(visionRadius * visionRadius - solX2 * solX2);

                    double dist1 = pow(solX1 - endX, 2) + pow(solY1 - endY, 2);
                    double dist2 = pow(solX2 - endX, 2) + pow(solY2 - endY, 2);

                    if (dist1 < dist2) {
                        //this means dist1 is the correct intersection
                        xPos = botX + solX1;
                        yPos = botY + solY1;
                    } else {
                        xPos = botX + solX2;
                        yPos = botY + solY2;
                    }

                } else if (discriminant == 0) { 
                    double sol = -b / 2 / a;
                    xPos = botX + sol;
                    yPos = botY + sqrt(visionRadius * visionRadius - sol * sol);

                }

            }
        }

        void setStage() {
            double distBotToEnd = sqrt(pow(path[stage].second.xPos - botX, 2) + pow(path[stage].second.yPos - botY, 2));
            
            if (distBotToEnd < visionRadius && stage < pathLength) {
                stage++;
                if (stage == pathLength) {
                    unlocked = false;
                } else computePathEq();
            } if (stage == pathLength) {
                unlocked = false;
            }
            
        }

    } ;
}
#endif