#ifndef TURTLEBOT_MAZE_WALL_DETECTION_H
#define TURTLEBOT_MAZE_WALL_DETECTION_H

#include <vector>
#include <cassert>
#include <algorithm>

#include "turtlebot_maze/geometric_utilities.h"

namespace turtlebot_maze{

    class WallDetection{

    public:

        WallDetection(double max_range);

        void updateWalls(const Pose& pose, const std::vector<double>& ranges, const std::vector<double>& angles);

        bool clearedWall(const Pose& pose, double& last_delta);

        void getWalls(WallModel& left_wall, WallModel& right_wall);

        void resetMedians();

    private:

        void fillAccumulator(const std::vector<double>& ranges, const std::vector<double>& angles);

        void findPeaks();

        void findEndPoints(const std::vector<double> &ranges, const std::vector<double> &angles);

        std::vector<std::vector<int>> accumulator_; // Hough transform accumulator for line detection

        std::vector<WallModel> walls_; // stored wall parameters

        std::vector<double> left_wall_xe_; // track wall endpoint estimates to find median
        std::vector<double> left_wall_ye_;
        std::vector<double> right_wall_xe_;
        std::vector<double> right_wall_ye_;

        const double laser_offset_ {M_PI_2}; // laser zeroth scan axis aligned with robot -y axis

        const double max_deviation_; // deviation from line model that signifies an endpoint
        const double max_range_diff_; // difference threshold between 2 adjacent ranges that indicates break in wall model
        const double max_range_; // max range for reliable wall endpoint detection
        const double theta_inc_; // angle resolution in radians
        const int grid_size_; // size of square accumulator matrix
        const double r_inc_; // range (distance) resolution in meters
        const int detect_threshold_; // min number of Hough votes (scan points) for a valid wall detection

    };

} // namespace turtlebot_maze

#endif //TURTLEBOT_MAZE_WALL_DETECTION_H
