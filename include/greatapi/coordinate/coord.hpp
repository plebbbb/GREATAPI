//Standard coord datatype, holds coords of arbritrary vector in R2
//provides transformation functions as well

#include "api.h"
#include "greatapi/angle_units/SRAD.hpp"
#pragma once

#ifndef coord_HPP
#define coord_HPP

namespace greatapi{

struct coord {
		double x;
		double y;
		double length;

		coord(std::pair<double, double> set) :x(set.first), y(set.second) { get_length(); }
		coord(std::tuple<double, double> set) :x(std::get<0>(set)), y(std::get<1>(set)) { get_length(); }
		coord() { x = 0; y = 0; length = 0;}

		//copy constructor
		coord operator=(std::pair<double, double> set) {
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
			return coord(std::pair<double,double>{xe,ye});
		}

		//updates length of coord
		double get_length() {
			length = double(sqrt(x * x + y * y));
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
	};
}
#endif
