#ifndef TURTLEBOT_MAZE_POSITION_HISTORY_H
#define TURTLEBOT_MAZE_POSITION_HISTORY_H

#include <map>

#include "turtlebot_maze/geometric_utilities.h"

namespace turtlebot_maze{

    class PositionHistory{
    public:
        void addPoint(const Point& pt);
        bool anyInRectangle(const Point& corner_a, const Point& corner_b) const;

    private:
        bool anyInRadius(const Point& center, double radius) const;

        // store of visited points in global frame, mapped by x-coordinate
        std::multimap<double, Point> visited_;

        // new point to add must be this far away from any point already in the history
        const double min_radius_ {0.5};
    };

} // namespace turtlebot_maze

#endif //TURTLEBOT_MAZE_POSITION_HISTORY_H
