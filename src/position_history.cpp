#include <iostream>
#include "../include/turtlebot_maze/position_history.h"

namespace turtlebot_maze{
    
    void PositionHistory::AddPoint(const Point& pt){
        visited_.insert(std::pair<double, Point>(pt.x, pt));
    }

    bool PositionHistory::AnyInRectangle(double left, double right, double lower, double upper){

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