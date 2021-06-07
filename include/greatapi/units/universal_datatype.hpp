#include<cmath>

#ifndef DATATYPE_HPP
#define DATATYPE_HPP

namespace greatapi {
  //standard angle units are RADIANS
  struct unit{
    double value;
    unit(int v):value(v){};

    //convert to double for math stuff
    //private:
      operator double(){ //this may lead to possible confusion between types
         return value;
      }

    //these should be hopefully overridden by angle and distance definitions. They are here to facilitate usage by control_loops
    void operator+=(unit increment) {
      value += (double)increment;
    }

    void operator-=(unit increment) {
      value -= (double)increment;
    }

    void operator/=(unit increment) {
      value /= (double)increment;
    }

    void operator*=(unit increment) {
      value *= (double)increment;
    }

    unit operator+(unit b) {
      return unit(value + b);
    }

    unit operator-(unit b) {
      return unit(value - b);
    }

    unit operator/(unit b) {
      return unit(value / b);
    }

    unit operator*(unit b) {
      return unit(value * b);
    }

    bool operator>(unit b){
      return value > b.value;
    }

    bool operator<(unit b){
      return value < b.value;
    }

    bool operator==(unit b){
      return value == b.value;
    }

    bool operator>=(unit b){
      return value >= b.value;
    }

    bool operator<=(unit b){
      return value <= b.value;
    }
  };
}

#endif
