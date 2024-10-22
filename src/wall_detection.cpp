#include "turtlebot_maze/wall_detection.h"

#include <iostream>

namespace turtlebot_maze{

    WallDetection::WallDetection(double max_range) :
    max_deviation_{0.2},
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

    void WallDetection::updateWalls(const turtlebot_maze::Pose &pose, const std::vector<double> &ranges,
                                    const std::vector<double> &angles) {
        // clear out previous wall models
        walls_.clear();

        // zero out the Hough accumulator
        for(auto& col : accumulator_)
            std::fill(col.begin(), col.end(), 0);

        fillAccumulator(ranges, angles);
        findPeaks();
        findEndPoints(ranges, angles);

        // transform wall models into the global frame
        for(auto i = 0; i < 2; ++i){
            walls_[i].p_c = transformToGlobal(walls_[i].p_c, pose);
            if(std::fabs(walls_[i].p_e.y) > 1e-3){ // TODO: use nan instead of zero check
                walls_[i].p_e = transformToGlobal(walls_[i].p_e, pose);
                if(i == 0){
                    right_wall_xe_.push_back(walls_[i].p_e.x);
                    right_wall_ye_.push_back(walls_[i].p_e.y);
                }else{
                    left_wall_xe_.push_back(walls_[i].p_e.x);
                    left_wall_ye_.push_back(walls_[i].p_e.y);
                }
            }
            walls_[i].a = wrapAngle(walls_[i].a + pose.h);
        }
    }


    void WallDetection::fillAccumulator(const std::vector<double> &ranges, const std::vector<double> &angles) {

        assert(ranges.size() == angles.size());

        for(auto j = 0; j < ranges.size(); ++j){
            double xj = ranges[j] * cos(angles[j] - laser_offset_);
            double yj = ranges[j] * sin(angles[j] - laser_offset_);
            for(int i = 0; i < grid_size_; ++i){
                double theta = (i - grid_size_ / 2) * theta_inc_;
                double r = xj * cos(theta) + yj * sin(theta);
                auto r_idx = static_cast<int> (r / r_inc_);
                if(r_idx >= -grid_size_ / 2 && r_idx < grid_size_ / 2)
                    accumulator_[i + 1][r_idx + grid_size_ / 2 + 1] += 1;
            }
        }
    }

    void WallDetection::findPeaks() {

        int count = 0;
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

                    if(count < 2){
                        walls_.emplace_back(WallModel{(i-1-grid_size_/2) * theta_inc_, (j-1-grid_size_/2) * r_inc_});
                        if(count == 1 && std::fabs(walls_[1].a + laser_offset_) < std::fabs(walls_[0].a + laser_offset_)){
                            std::swap(walls_[0], walls_[1]); //wall[0].a (right wall) should be closest to -pi/2
                        }
                        ++count;
                    }else{
                        // find wall on robot's right that is best aligned with robot's x-axis
                        if(std::fabs((i-1-grid_size_/2)*theta_inc_ + laser_offset_) < std::fabs(walls_[0].a + laser_offset_)){
                            walls_[0].a = (i-1-grid_size_/2) * theta_inc_;
                            walls_[0].r = (j-1-grid_size_/2) * r_inc_;
                        }
                        // find wall on robot's left that is best aligned with robot's y-axis
                        if(std::fabs((i-1-grid_size_/2)*theta_inc_ - laser_offset_) < std::fabs(walls_[1].a - laser_offset_)){
                            walls_[1].a = (i-1-grid_size_/2) * theta_inc_;
                            walls_[1].r = (j-1-grid_size_/2) * r_inc_;
                        }
                    }
                }
            }
        }

        // if the left or right wall model is more than 45 deg off of the robot's x axis, don't use it
        if(std::fabs(walls_[0].a + laser_offset_) > M_PI_4)
            walls_[0].r = 0;
        if(std::fabs(walls_[1].a - laser_offset_) > M_PI_4)
            walls_[1].r = 0;
    }

    void WallDetection::findEndPoints(const std::vector<double> &ranges, const std::vector<double> &angles) {

        for(auto ii = 0; ii < 2; ++ii){ // left wall, right wall

            double ang = walls_[ii].a + laser_offset_; // [0, pi]
            // angles should be sorted in increasing order
            auto it = std::lower_bound(angles.begin(), angles.end(), ang);
            int idx = std::min(static_cast<int>(std::distance(angles.begin(), it)), static_cast<int>(angles.size() - 1));
            //assert(idx >= 0 && idx < angles.size()); // something wrong if not in the array

            int incr = ii == 0 ? 1 : -1; // direction to travel along scan
            int end_idx = ii == 0 ? static_cast<int>(angles.size()) : -1; // where to stop

            // walk up the scan for right wall, down the scan for left wall
            for(int i = idx + incr; i != end_idx; i += incr){
                if(ranges[i] > max_range_){
                    // don't estimate endpoints from ranges that are too long
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
                    break;
                }
            }

            // closest wall point is the one at the robot's side
            int e = ii == 0 ? 0 : static_cast<int>(angles.size() - 1);
            double error_delta = ranges[e] * cos(angles[e] - ang) - walls_[ii].r;
            walls_[ii].p_c.x = ranges[e] * cos(angles[e] - laser_offset_) - error_delta * cos(ang);
            walls_[ii].p_c.y = ranges[e] * sin(angles[e] - laser_offset_) - error_delta * sin(ang);
        }
    }

    void WallDetection::getWalls(turtlebot_maze::WallModel &left_wall, turtlebot_maze::WallModel &right_wall) {

        const int min_samples = 5;
        right_wall.r = walls_[0].r; right_wall.a = walls_[0].a;
        left_wall.r = walls_[1].r; left_wall.a = walls_[1].a;
        // TODO: should r and a be median as well?
        right_wall.p_c = walls_[0].p_c;
        left_wall.p_c = walls_[1].p_c;

        // find medians of stored coordinates of far endpoints so as to provide reliable values
        if(!right_wall_xe_.empty() && !right_wall_ye_.empty()) {
            auto n = right_wall_xe_.size();
            auto m = right_wall_ye_.size();
            if(n >= min_samples && m >= min_samples) {
                std::nth_element(right_wall_xe_.begin(), right_wall_xe_.begin() + n / 2, right_wall_xe_.end());
                std::nth_element(right_wall_ye_.begin(), right_wall_ye_.begin() + m / 2, right_wall_ye_.end());
                right_wall.p_e = Point{right_wall_xe_[n / 2], right_wall_ye_[m / 2]};
            }
        }
        if(!left_wall_xe_.empty() && !left_wall_ye_.empty()){
            auto n = left_wall_xe_.size();
            auto m = left_wall_ye_.size();
            if(n >= min_samples && m >= min_samples){
                std::nth_element(left_wall_xe_.begin(), left_wall_xe_.begin() + n/2, left_wall_xe_.end());
                std::nth_element(left_wall_ye_.begin(), left_wall_ye_.begin() + m/2, left_wall_ye_.end());
                left_wall.p_e = Point{left_wall_xe_[n/2], left_wall_ye_[m/2]};
            }
        }
    }

    void WallDetection::resetMedians() {
        left_wall_xe_.clear();
        left_wall_ye_.clear();
        right_wall_xe_.clear();
        right_wall_ye_.clear();
    }

} // namespace turtlebot_maze
