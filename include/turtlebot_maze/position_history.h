#ifndef TURTLEBOT_MAZE_POSITION_HISTORY_H
#define TURTLEBOT_MAZE_POSITION_HISTORY_H

#include <map>

#include "turtlebot_maze/geometric_utilities.h"

namespace turtlebot_maze{

    class PositionHistory{
    public:
        void addPoint(const Point& pt);
        bool anyInRectangle(const Point& upper, const Point& lower);
        void printPoints();

    private:
        bool anyInRadius(const Point& point, double radius);
        bool anyInRectangleHelper(double left, double right, double lower, double upper);
        // store of visited points in global frame, mapped by x-coordinate
        std::multimap<double, Point> visited_;
    };

} // namespace turtlebot_maze

#endif //TURTLEBOT_MAZE_POSITION_HISTORY_H
