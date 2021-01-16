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

    double distance(const Point& a, const Point& b){
        return sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
    }

    // rotate and translate a point in the robot frame to the global frame
    Point transformToGlobal(const Point& p_r, const Pose& pose){
        Point p_i;
        double laser_heading = 0.0; // laser faces forward but its x-axis has been defined at range 0
        p_i.x = p_r.x * cos(pose.h - laser_heading) - p_r.y * sin(pose.h - laser_heading) + pose.p.x;
        p_i.y = p_r.x * sin(pose.h - laser_heading) + p_r.y * cos(pose.h - laser_heading) + pose.p.y;
        return p_i;
    }

    // constrain angle to [-pi, pi]
    double wrapAngle(double angle){
        while(angle < -M_PI)
            angle += 2.0*M_PI;
        while(angle > M_PI)
            angle -= 2.0*M_PI;
        return angle;
    }

    // handle the discontinuity at +- pi when subtracting b from a
    double angleDifference(double a, double b){
        double result = a - b;
        if(result > M_PI) result -= 2*M_PI;
        if(result < -M_PI) result += 2*M_PI;
        return result;
    }

} // namespace turtlebot_maze

#endif //TURTLEBOT_MAZE_GEOMETRIC_UTILITIES_H
