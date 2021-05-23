//Standard Coord datatype, holds Coords of arbritrary vector in R2
//provides transformation functions as well

#include "api.h"
#include "greatapi/angle_units/SRAD.hpp"
#pragma once

#ifndef COORD_HPP
#define COORD_HPP

struct Coord {
		double x;
		double y;
		double length;
		Coord(std::pair<double, double> set) :x(set.first), y(set.second) { get_length(); }
		Coord(std::tuple<double, double> set) :x(std::get<0>(set)), y(std::get<1>(set)) { get_length(); }

		//copy constructor
		Coord operator=(std::pair<double, double> set) {
			return Coord(set);
		}
		Coord() { x = 0; y = 0; }

		//relative Coord calculation from two position
		Coord(Coord initial, Coord final):x(final.x - initial.x),y(final.y - initial.y){
			get_length();
		}

		/******************************************************************************/
		//Utility functions

		void self_transform_matrix(SRAD offset){
			x = cos(offset)*x + sin(offset)*x;
			y = - sin(offset)*y + cos(offset)*y;
		}

		Coord transform_matrix(SRAD offset){
			double xe = double(cos((double)offset)*(double)x + sin((double)offset)*(double)y);
			double ye = double(cos((double)offset)*(double)y - sin((double)offset)*(double)x);
			return Coord(std::pair<double,double>{xe,ye});
		}

		//updates length of Coord
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
			return "Coord: " + ss.str() + "\tx=" + std::to_string(x.value) +
								"\ty=" + std::to_string(y.value) + "\tlength=" + std::to_string(length.value) + "\n";
		}
*/
		/******************************************************************************/
		//Manipulation functions
		/*Remember that an conversion from pairs({x,y}) to Coord can automatically happen
		Using that instead of an actual Coord type is also perfectly legal*/
		void operator+=(Coord change) {
			x += change.x;
			y += change.y;
			get_length();
		}

		void operator-=(Coord change) {
			x -= change.x;
			y -= change.y;
			get_length();
		}
	};
#endif
