#ifndef NODE_HPP
#define NODE_HPP

namespace purePursuit {
    struct Node {
        long double xPos = 0;
        long double yPos = 0;

        Node(double xPos_, double yPos_) {
            xPos = xPos_;
            yPos = yPos_;
        }
        Node() {}
    } ;
}
#endif