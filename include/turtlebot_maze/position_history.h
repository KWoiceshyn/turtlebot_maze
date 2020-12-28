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
        bool AnyInRectangleHelper(double left, double right, double lower, double upper);
        std::multimap<double, Point> visited_;
    };
}

#endif //TURTLEBOT_MAZE_POSITION_HISTORY_H
