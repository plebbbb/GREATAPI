//Standard coord datatype, holds coords of arbritrary vector in R2
//provides transformation functions as well

#include "api.h"
#include "greatapi/units/angle_units/srad.hpp"
#include "greatapi/units/distance_units/universal_distance.hpp"

#pragma once

#ifndef coord_HPP
#define coord_HPP

namespace greatapi{

struct coord {
		distance x;
		distance y;
		distance length;

		coord(distance xd, distance yd) :x(xd), y(yd) { get_length(); }
		coord(std::pair<distance, distance> set) :x(set.first), y(set.second) { get_length(); }
		coord(std::tuple<distance, distance> set) :x(std::get<0>(set)), y(std::get<1>(set)) { get_length(); }
		coord() { x = 0; y = 0; length = 0;}

		//copy constructor
		coord operator=(std::pair<distance, distance> set) {
			return coord(set);
		}

		//relative coord calculation from two position
		coord(coord initial, coord final):x(final.x - initial.x),y(final.y - initial.y){
			get_length();
		}

		/******************************************************************************/
		//Utility functions

		void self_transform_matrix(SRAD offset){
			x = cos(offset)*x + sin(offset)*x;
			y = - sin(offset)*y + cos(offset)*y;
		}

		coord transform_matrix(SRAD offset){
			double xe = double(cos((double)offset)*(double)x + sin((double)offset)*(double)y);
			double ye = double(cos((double)offset)*(double)y - sin((double)offset)*(double)x);
			return coord(std::pair<distance,distance>{xe,ye});
		}

		//updates length of coord
		distance get_length() {
			length = sqrt( (x * x) + (y * y)); //I have no idea if bedmas is still legal, assume it is not.
			return length;
		}

		/******************************************************************************/
		//Conversion function
	 	operator std::pair<double, double>() {
	 		return { x,y };
		}
/*
		std::string debug() {
			const void * tmp = static_cast<const void*>(this);
			std::stringstream ss;
			ss << tmp;
			return "coord: " + ss.str() + "\tx=" + std::to_string(x.value) +
								"\ty=" + std::to_string(y.value) + "\tlength=" + std::to_string(length.value) + "\n";
		}
*/
		/******************************************************************************/
		//Manipulation functions
		/*Remember that an conversion from pairs({x,y}) to coord can automatically happen
		Using that instead of an actual coord type is also perfectly legal*/
		void operator+=(coord change) {
			x += change.x;
			y += change.y;
			get_length();
		}

		void operator-=(coord change) {
			x -= change.x;
			y -= change.y;
			get_length();
		}

		coord operator+(coord change) {
			return coord((x+change.x), (y+change.y));
		}

		coord operator-(coord change) {
			return coord((x-change.x), (y-change.y));
		}

		//DOT PRODUCT OPERATOR
		//You learn this in 11ap/12 calc. The most frequent usage you will see will be to get the angle between two vectors
		//see documentation site on how this works.
		double operator*(coord change) {
			return (x*change.x) + (y*change.y);
		}

		coord operator*(double constant){
			return coord(x*constant, y*constant);
		}
	};
}
#endif
