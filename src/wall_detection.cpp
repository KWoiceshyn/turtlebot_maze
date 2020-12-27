#include <cassert>
#include <iostream>
#include "../include/turtlebot_maze/wall_detection.h"

namespace turtlebot_maze{

    WallDetection::WallDetection(double max_range) :
    max_deviation_{0.12},
    theta_inc_{2.0 * M_PI / 180.0},
    grid_size_{static_cast<std::size_t> (M_PI / theta_inc_)},
    r_inc_{max_range / grid_size_},
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
        for(auto& wall : walls_){
            if(Distance(pose.p, wall.p_c) > Distance(pose.p, wall.p_e)){
                std::swap(wall.p_c, wall.p_e);
            }
            //std::cout << wall.a <<" "<<wall.r<<" "<<wall.p_c.x<<" "<<wall.p_c.y<<" "<<wall.p_e.x<<" "<<wall.p_e.y<<std::endl;
        }

        // convert wall models into the global frame
        for(auto& wall : walls_){
            wall.p_c = TransformToGlobal(wall.p_c, pose);
            wall.p_e = TransformToGlobal(wall.p_e, pose);
            wall.a = WrapAngle(wall.a + pose.h);
            //std::cout << "transformed "<< wall.a <<" "<<wall.r<<" "<<wall.p_c.x<<" "<<wall.p_c.y<<" "<<wall.p_e.x<<" "<<wall.p_e.y<<std::endl;
        }

    }


    void WallDetection::FillAccumulator(const std::vector<double> &ranges, const std::vector<double> &angles) {

        assert(ranges.size() == angles.size());

        for(auto i = 0; i < ranges.size(); ++i){
            double xi = ranges[i] * cos(angles[i]);
            double yi = ranges[i] * sin(angles[i]);
            for(int j = 0; j < grid_size_; ++j){
                double theta = j*theta_inc_;
                double r = std::fabs(xi * cos(theta) + yi * sin(theta));
                auto r_idx = static_cast<int> (r / r_inc_);
                if(r_idx >= 0 && r_idx < grid_size_)
                    accumulator_[j + 1][r_idx + 1] += 1;
            }
        }
    }

    void WallDetection::FindPeaks() {

        int count = 0;
        for(auto i = 1; i <= grid_size_; ++i){
            for(auto j = 1; j <= grid_size_; ++j){
                if(accumulator_[i][j] >= detect_threshold_ && // check greater than 8 neighbors
                   accumulator_[i][j] >= accumulator_[i-1][j] &&
                   accumulator_[i][j] >= accumulator_[i+1][j] &&
                   accumulator_[i][j] >= accumulator_[i][j-1] &&
                   accumulator_[i][j] >= accumulator_[i][j+1] &&
                   accumulator_[i][j] >= accumulator_[i-1][j-1] &&
                   accumulator_[i][j] >= accumulator_[i-1][j+1] &&
                   accumulator_[i][j] >= accumulator_[i+1][j-1] &&
                   accumulator_[i][j] >= accumulator_[i+1][j+1]){
                    if(count < 2){
                        walls_.emplace_back(WallModel{i * theta_inc_, j * r_inc_});
                        if(count == 1){
                            if(walls_[1].a < walls_[0].a)
                                std::swap(walls_[0], walls_[1]);
                        }
                        ++count;
                    }else{
                        if(std::fabs(i*theta_inc_) < std::fabs(walls_[0].a)){ // find wall that is closest to robot's right
                            walls_[0].a = i * theta_inc_;
                            walls_[0].r = j * r_inc_;
                        }
                        if(std::fabs(M_PI - i*theta_inc_) < std::fabs(M_PI - walls_[1].r)){ // find the wall that is closest to robot's left
                            walls_[1].a = i * theta_inc_;
                            walls_[1].r = j * r_inc_;
                        }
                    }
                }
            }
        }
    }

    void WallDetection::FindEndPoints(const std::vector<double> &ranges, const std::vector<double> &angles) {

        for(auto& wall : walls_){
            // angles should be sorted in increasing order
            auto it = std::lower_bound(angles.begin(), angles.end(), wall.a);
            int idx = std::distance(angles.begin(), it);
            assert(idx >= 0 && idx < angles.size()); // something wrong if not in the array
            bool found_end = false;

            // walk down the scan
            for(int i = idx - 1; i >= 0; --i){
                double error  = ranges[i] * cos(angles[i] - wall.a) - wall.r;
                if(std::fabs(error) > max_deviation_){
                    // use the previous good point
                    double still_good_error = ranges[i+1] * cos(angles[i+1] - wall.a) - wall.r;
                    // assign endpoints arbitrarily here
                    // force endpoint to lie on line defined by wall model
                    wall.p_c.x = ranges[i+1] * cos(angles[i+1]) - still_good_error * cos(wall.a);
                    wall.p_c.y = ranges[i+1] * sin(angles[i+1]) - still_good_error * sin(wall.a);
                    found_end = true;
                    break;
                }
            }
            if(!found_end){ // endpoint is start of scan
                double error_delta = ranges[0] * cos(angles[0] - wall.a) - wall.r;
                wall.p_c.x = ranges[0] * cos(angles[0]) - error_delta * cos(wall.a);
                wall.p_c.y = ranges[0] * sin(angles[0]) - error_delta * sin(wall.a);
            }

            // walk up the scan
            found_end = false;
            for(int i = idx + 1; i < angles.size(); ++i){
                double error  = ranges[i] * cos(angles[i] - wall.a) - wall.r;
                if(std::fabs(error) > max_deviation_){
                    // use the previous good point
                    double still_good_error = ranges[i-1] * cos(angles[i-1] - wall.a) - wall.r;
                    // assign endpoints arbitrarily here
                    // force endpoint to lie on line defined by wall model
                    wall.p_e.x = ranges[i-1] * cos(angles[i-1]) - still_good_error * cos(wall.a);
                    wall.p_e.y = ranges[i-1] * sin(angles[i-1]) - still_good_error * sin(wall.a);
                    found_end = true;
                    break;
                }
            }
            if(!found_end){ // endpoint is end of scan
                auto e = angles.size() - 1;
                double error_delta = ranges[e] * cos(angles[e] - wall.a) - wall.r;
                wall.p_e.x = ranges[e] * cos(angles[e]) - error_delta * cos(wall.a);
                wall.p_e.y = ranges[e] * sin(angles[e]) - error_delta * sin(wall.a);
            }
        }
    }

    const std::vector<WallModel>& WallDetection::GetWalls() {
        return walls_;
    }

}
