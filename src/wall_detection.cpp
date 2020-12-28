#include <cassert>
#include <iostream>
#include <algorithm>
#include "../include/turtlebot_maze/wall_detection.h"

namespace turtlebot_maze{

    WallDetection::WallDetection(double max_range) :
    max_deviation_{0.12},
    max_range_diff_{1.2},
    max_range_{max_range},
    theta_inc_{2.0 * M_PI / 180.0},
    grid_size_{static_cast<int> (2.0 * M_PI / theta_inc_)}, // [-pi, pi]
    r_inc_{2.0 * max_range / grid_size_}, // positive and negative ranges
    detect_threshold_{40}
    {
        accumulator_.resize(grid_size_ + 2); // pad with zeros
        for(auto& col : accumulator_)
            col.resize(grid_size_ + 2, 0);
    }

    void WallDetection::UpdateWalls(const turtlebot_maze::Pose &pose, const std::vector<double> &ranges,
                                    const std::vector<double> &angles) {
        // clear out previous wall models
        walls_.clear();

        // zero out the Hough accumulator
        for(auto& col : accumulator_)
            std::fill(col.begin(), col.end(), 0);

        // fill the Hough accumulator using the angles and ranges from laser scan
        FillAccumulator(ranges, angles);

        // find the peaks (valid Wall models) in the Hough accumulator
        FindPeaks();

        // find the end points (e.g. corners) of the detected walls
        FindEndPoints(ranges, angles);

        // choose wall endpoints based on proximity to robot pose
        /*for(auto& wall : walls_){
            if(Distance(pose.p, wall.p_c) > Distance(pose.p, wall.p_e)){
                std::swap(wall.p_c, wall.p_e);
            }
            //std::cout <<"robot frame a "<< wall.a <<" r "<<wall.r<<" x0 "<<wall.p_c.x<<" y0 "<<wall.p_c.y<<" x1 "<<wall.p_e.x<<" y1 "<<wall.p_e.y<<std::endl;
        }*/

        // convert wall models into the global frame
        for(auto& wall : walls_){
            //wall.p_c = TransformToGlobal(wall.p_c, pose);
            if(std::fabs(wall.p_e.y) > 0.1){
                wall.p_e = TransformToGlobal(wall.p_e, pose);
            }
            wall.a = WrapAngle(wall.a + pose.h);
            //std::cout << "transformed a "<< wall.a <<" r "<<wall.r<<" x0 "<<wall.p_c.x<<" y0 "<<wall.p_c.y<<" x1 "<<wall.p_e.x<<" y1 "<<wall.p_e.y<<std::endl;
        }

    }


    void WallDetection::FillAccumulator(const std::vector<double> &ranges, const std::vector<double> &angles) {

        assert(ranges.size() == angles.size());

        for(auto j = 0; j < ranges.size(); ++j){
            double xj = ranges[j] * cos(angles[j] - laser_offset_);
            double yj = ranges[j] * sin(angles[j] - laser_offset_);
            for(int i = 0; i < grid_size_; ++i){
                double theta = (i - grid_size_/2)*theta_inc_;
                double r = xj * cos(theta) + yj * sin(theta);
                auto r_idx = static_cast<int> (r / r_inc_);
                if(r_idx >= -grid_size_/2 && r_idx < grid_size_/2)
                    accumulator_[i + 1][r_idx + grid_size_/2 + 1] += 1;
            }
        }
    }

    void WallDetection::FindPeaks() {

        int count = 0;
        std::vector<Point> record;
        for(int i = 1; i <= grid_size_; ++i){
            for(int j = 1; j <= grid_size_; ++j){
                if(accumulator_[i][j] >= detect_threshold_ && // check greater than 8 neighbors
                   accumulator_[i][j] >= accumulator_[i-1][j] &&
                   accumulator_[i][j] >= accumulator_[i+1][j] &&
                   accumulator_[i][j] >= accumulator_[i][j-1] &&
                   accumulator_[i][j] >= accumulator_[i][j+1] &&
                   accumulator_[i][j] >= accumulator_[i-1][j-1] &&
                   accumulator_[i][j] >= accumulator_[i-1][j+1] &&
                   accumulator_[i][j] >= accumulator_[i+1][j-1] &&
                   accumulator_[i][j] >= accumulator_[i+1][j+1] &&
                   j >= grid_size_/2){ // only want positive range values
                    record.emplace_back(Point{(i-1-grid_size_/2) * theta_inc_, (j-1-grid_size_/2) * r_inc_});
                    if(count < 2){
                        walls_.emplace_back(WallModel{(i-1-grid_size_/2) * theta_inc_, (j-1-grid_size_/2) * r_inc_});
                        if(count == 1 && std::fabs(walls_[1].a + laser_offset_) < std::fabs(walls_[0].a + laser_offset_)){
                            std::swap(walls_[0], walls_[1]); //wall[0] should be closest to -pi/2
                        }
                        ++count;
                    }else{
                        if(std::fabs((i-1-grid_size_/2)*theta_inc_ + laser_offset_) < std::fabs(walls_[0].a + laser_offset_)){ // find wall that is closest to robot's right
                            walls_[0].a = (i-1-grid_size_/2) * theta_inc_;
                            walls_[0].r = (j-1-grid_size_/2) * r_inc_;
                        }
                        if(std::fabs((i-1-grid_size_/2)*theta_inc_ - laser_offset_) < std::fabs(walls_[1].a - laser_offset_)){ // find wall that is closest to robot's left
                            walls_[1].a = (i-1-grid_size_/2) * theta_inc_;
                            walls_[1].r = (j-1-grid_size_/2) * r_inc_;
                        }
                    }
                }
            }
        }
        if(std::fabs(walls_[0].a + laser_offset_) > 0.1 || std::fabs(walls_[1].a - laser_offset_) > 0.1){
            std::cout << "Wall list:\n";
            for(const auto& p : record)
                std::cout << "a " << p.x << " r " << p.y << "\n";
        }
        //std::sort(walls_.begin(), walls_.end(), [](const WallModel& lhs, const WallModel& rhs){ return lhs.a < rhs.a;});
    }

    void WallDetection::FindEndPoints(const std::vector<double> &ranges, const std::vector<double> &angles) {

        for(auto ii = 0; ii < 2; ++ii){

            double ang = walls_[ii].a + laser_offset_; // [0, pi]
            // angles should be sorted in increasing order
            auto it = std::lower_bound(angles.begin(), angles.end(), ang);
            int idx = std::min(static_cast<int>(std::distance(angles.begin(), it)), static_cast<int>(angles.size() - 1));
            assert(idx >= 0 && idx < angles.size()); // something wrong if not in the array
            bool found_end = false;

            int incr = ii == 0 ? 1 : -1;
            int end_idx = ii == 0 ? angles.size() : -1;

            // walk up the scan for right wall, down the scan for left wall
            for(int i = idx + incr; i != end_idx; i += incr){
                if(ranges[i] > max_range_){
                    // don't estimate endpoints from ranges that are too long
                    found_end = true;
                    break;
                }
                double error  = ranges[i] * cos(angles[i] - ang) - walls_[ii].r;
                if(std::fabs(error) > max_deviation_ ||
                   std::fabs(ranges[i] - ranges[i-incr]) > max_range_diff_){
                    // use the previous good point
                    double still_good_error = ranges[i-incr] * cos(angles[i-incr] - ang) - walls_[ii].r;
                    // force endpoint to lie on line defined by wall model
                    walls_[ii].p_e.x = ranges[i-incr] * cos(angles[i-incr] - laser_offset_) - still_good_error * cos(ang);
                    walls_[ii].p_e.y = ranges[i-incr] * sin(angles[i-incr] - laser_offset_) - still_good_error * sin(ang);
                    found_end = true;
                    break;
                }
            }
            if(!found_end){ // endpoint is end of scan
                int e = end_idx - incr;
                double error_delta = ranges[e] * cos(angles[e] - ang) - walls_[ii].r;
                walls_[ii].p_e.x = ranges[e] * cos(angles[e] - laser_offset_) - error_delta * cos(ang);
                walls_[ii].p_e.y = ranges[e] * sin(angles[e] - laser_offset_) - error_delta * sin(ang);
            }
        }
    }

    const std::vector<WallModel>& WallDetection::GetWalls() {
        return walls_;
    }

}
