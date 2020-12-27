#include "../include/turtlebot_maze/geometric_utilities.h"

namespace turtlebot_maze{

    double Distance(const Point& a, const Point& b){
        return sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
    }

    Point TransformToGlobal(const Point& p_r, const Pose& pose){
        Point p_i;
        p_i.x = p_r.x * cos(pose.h) - p_r.y * sin(pose.h) + pose.p.x;
        p_i.y = p_r.x * sin(pose.h) + p_r.y * cos(pose.h) + pose.p.y;
        return p_i;
    }

    double WrapAngle(double angle){
        while(angle < -M_PI)
            angle += 2.0*M_PI;
        while(angle > M_PI)
            angle -= 2.0*M_PI;
        return angle;
    }

}

