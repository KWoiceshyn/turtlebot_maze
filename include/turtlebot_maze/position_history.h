#ifndef TURTLEBOT_MAZE_POSITION_HISTORY_H
#define TURTLEBOT_MAZE_POSITION_HISTORY_H

#include "geometric_utilities.h"

#include <map>

namespace turtlebot_maze{

    class PositionHistory{
    public:
        void AddPoint(const Point& pt);
        bool AnyInRectangle(const Point& upper, const Point& lower);
        void PrintPoints();

    private:
        bool AnyInRadius(const Point& point, double radius);
        bool AnyInRectangleHelper(double left, double right, double lower, double upper);
        std::multimap<double, Point> visited_; // store of visited points in global frame, mapped by x-coordinate
    };
}

#endif //TURTLEBOT_MAZE_POSITION_HISTORY_H
