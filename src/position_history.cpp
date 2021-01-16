#include "turtlebot_maze/position_history.h"


namespace turtlebot_maze{
    
    void PositionHistory::addPoint(const Point& pt){

        if(!anyInRadius(pt, min_radius_))
            visited_.insert(std::pair<double, Point>{pt.x, pt});
    }

    bool PositionHistory::anyInRectangle(const turtlebot_maze::Point &corner_a, const turtlebot_maze::Point &corner_b) const {

        // assume rectangle is axis-aligned but we don't know which opposite corner points are given
        double upper_edge = std::max(corner_a.y, corner_b.y);
        double lower_edge = std::min(corner_a.y, corner_b.y);
        double right_edge = std::max(corner_a.x, corner_b.x);
        double left_edge = std::min(corner_a.x, corner_b.x);

        auto it_left = visited_.lower_bound(left_edge); // index of left edge of rectangle
        auto it_right = visited_.upper_bound(right_edge); // index of right edge of rectangle

        while(it_left != it_right && it_left != visited_.end()){

            // if in x-range and also in y-range
            if(it_left->second.y > lower_edge && it_left->second.y < upper_edge)
                return true;
            ++it_left;
        }
        return false;
    }

    bool PositionHistory::anyInRadius(const turtlebot_maze::Point &center, double radius) const {

        auto it_left = visited_.lower_bound(center.x - radius); // index of left extent of circle
        auto it_right = visited_.upper_bound(center.x + radius); // index of right extent of circle

        while(it_left != it_right && it_left != visited_.end()){

            double dx = std::fabs(it_left->second.x - center.x);
            double dy = dx < radius ? sqrt(pow(radius, 2.0) - pow(dx, 2.0)) : 0.0;

            if(std::fabs(it_left->second.y - center.y) < dy)
                return true;
            ++it_left;
        }
        return false;
    }
    
} // namespace turtlebot_maze