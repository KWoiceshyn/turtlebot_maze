#include <iostream>
#include "../include/turtlebot_maze/position_history.h"

namespace turtlebot_maze{
    
    void PositionHistory::AddPoint(const Point& pt){
        visited_.insert(std::pair<double, Point>(pt.x, pt));
    }

    bool PositionHistory::AnyInRectangle(const turtlebot_maze::Point &upper, const turtlebot_maze::Point &lower) {

        double upper_bound = std::max(upper.y, lower.y);
        double lower_bound = std::min(upper.y, lower.y);
        double right_bound = std::max(upper.x, lower.x);
        double left_bound = std::min(upper.x, lower.x);
        return AnyInRectangleHelper(left_bound, right_bound, lower_bound, upper_bound);
    }

    bool PositionHistory::AnyInRectangleHelper(double left, double right, double lower, double upper){

        std::cout << "Checking rect: L " << left << " R " << right << " L " << lower << " U " << upper << "\n";
        auto it_left = visited_.lower_bound(left);
        auto it_right = visited_.upper_bound(right);

        while(it_left != it_right && it_left != visited_.end()){

            if(it_left->second.y > lower && it_left->second.y < upper)
                return true;
            ++it_left;
        }
        return false;
    }

    void PositionHistory::PrintPoints() {
        std::cout<<"Visited: ";
        for(const auto& m : visited_){
            std::cout << m.second.x << " " << m.second.y << "\n";
        }
    }
    
}