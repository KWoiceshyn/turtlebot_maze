#ifndef TURTLEBOT_MAZE_GEOMETRIC_UTILITIES_H
#define TURTLEBOT_MAZE_GEOMETRIC_UTILITIES_H

#include <cmath>

namespace turtlebot_maze {

    struct Point{

        Point(double xx = 0.0, double yy = 0.0){
            x = xx;
            y = yy;
        }

        double x;
        double y;
    };

    struct Pose{
        Point p; // position
        double h; // heading
    };

    struct WallModel {

        WallModel(double aa = 0.0, double rr = 0.0){
            a = aa;
            r = rr;
            p_c.x = 0; p_c.y = 0; p_e.x = 0; p_e.y = 0;
        }

        double a; // orientation of wall in global frame
        double r; // min distance from wall to scanner at time of detection
        Point p_c; // global x-y coordinate of wall point that is closest to robot when wall is detected
        Point p_e; // global x-y coordinate of wall point that is furthest from robot when wall is detected
    };

    double Distance(const Point& a, const Point& b);

    // rotate and translate a point in the robot frame to the global frame
    Point TransformToGlobal(const Point& p_r, const Pose& pose);

    // constrain angle to [-pi, pi]
    double WrapAngle(double angle);

    // handle the discontinuity at +- pi when subtracting b from a
    double AngleDifference(double a, double b);

} // turtlebot_maze

#endif //TURTLEBOT_MAZE_GEOMETRIC_UTILITIES_H
