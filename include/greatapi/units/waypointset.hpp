#include<cmath>
#include "greatapi/basic_datatypes.hpp"
#include "greatapi/units/waypoint_generator/universal_generator.hpp"
#include "greatapi/units/waypoint_generator/bezier.hpp"

#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

namespace greatapi {
  //apparently this is legal?
  //ngl, not sure if we want to integrate PP support directly into this
  struct waypointset : public std::vector<coord> {
    waypointset(std::vector<coord> coordinateset):vector<coord>(coordinateset){}

    //X has an interval between 0 and 1.
    virtual SRAD get_heading(double scalefactor);

    //return rise/run slope for easy negative recoprical creation
    virtual double get_slope(double scalefactor);

    //TBD: develop an approach to sepeprate this data
    /*
      As an example, my previous code had a percentage based structure,
      where each set of waypoints consists of a movement, where progress
      spans from 0% completion to 100%. I would have other operations, like scoring,
      span an interval of this 0-100 space. However, I only had a single waypoint.

      A multi-point version of this design would probably distribute the line betwee
      each subsequent waypoint, and assign it an interval of perentage completion.
      For example, the second line in a path of multiple lines might be the
      range of 10% to 20% of the overall path completion which is returned.

      You can calculate the lengths of each line, and compare them to the total
      to determine the ranges, though you could possibly get away with assuming they are all the same length.

      As for determining which line you are on, and how far along you are on it, I'll provide this idea:

      You could perhaps make a line that extends perpendicularly from each path line.
      From there, compute that line's closest point to the bot at its start and end

      if the direction each closest point travels is opposite(i.e up-left on start, down-right on end),
      it is safe to assume that the bot is somewhere between.

      To do the above, solve for the closest distance between the bot and the perp line. The process of doing this requires you to get
      the closest point on the perpendicular line. you can determine the direction one must travel in each axis by subtracting the bot coord from
      that closest point.

      Compare the travels on a line by line basis, until the travel directions are opposite.
      It is plausable that there exist edge cases(like a really sharp eclipse) where multiple valid solutions exist. Calculate them all, and select the one with the
      shortest distance to center. The other edge case of a loop where a path overlaps twice can be solved by some sort of line segement counting done probably externally.

      There exists the question of which line's slope to use for each point. Please keep consistency, and ensure that no interval of space isn't part of a line.
      I would actually suggest averaging out the slope for perp lines extending out of a point connected to two lines. If one were to use each slope and
      calculate twice, once for each line segement, there exists a deadzone where the bot isn't assigned a line. So yeah, for any points connecting to two other points,
      have your perp. line extend out at avg slope of the two lines it forms.

      Upon hitting the last points where these travel directions are the same as their respective starts,

      Make an equation using the slope of the (actual, like negative recoprical actual) perpendicular of the line between these last points, which intercepts
      specific parts of the line between the two points. Check how far that line equation is from the bot's odom position,
      and try and solve for the specific point on the main line which must intercept for the perpendicular line to cross the bot's positon.
      Use some kinda solving algo, like newton-rasphaodian, or even a modified binary search style system, where you go up/down
      the line depending on which direction you are off in.
      (for that approach, halve the change each time to prevent instability)
      iterate a few loops of this, and the interception point can be considered where you are on the path.

      After taking a look at the solving algo, you may notice the problem of how the line interval isnt fully covered by this solution
      think of our line inteval as a triangle, and the range we can solve for a rectangle of the same height and same width as the triangle.
      If one were to overlay these shapes, theres always going to be an area where you aren't in both of the shapes' areas, but only one.

      use your common sense and figure out what problems exist with those when your bot isnt in both

      The soltuion, is that the moment which you exceed this interval, you modify your slope so that your "perpendicular" line is still within the realm
      of reality. Basically, if you need to go past endpoints, just change the slope of your line so that you intercept them.

      I'm just eyeballing this, so it might be possible to legitimately solve this series of equations without resorting to algos. If so, just do that.

      -Isaac
    */
  };

  struct Gwaypointset : public waypointset {
    WaypointGenerator generator;
    Gwaypointset(WaypointGenerator e, int density):waypointset(e.compute(density)),generator(e){}

    SRAD get_heading(double scalefactor){
      return generator.computeheading(scalefactor);
    }

    double get_slope(double scalefactor){
      return generator.computeslope(scalefactor);
    }
  };

}

#endif
