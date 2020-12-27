#ifndef TURTLEBOT_MAZE_WALL_DETECTION_H
#define TURTLEBOT_MAZE_WALL_DETECTION_H

#include "geometric_utilities.h"

#include <vector>
#include <cmath>

namespace turtlebot_maze{

    class WallDetection{

    public:

        WallDetection(double max_range);

        void UpdateWalls(const Pose& pose, const std::vector<double>& ranges, const std::vector<double>& angles);

        double DistToLeftWall(const Pose& pose);

        double HeadingError(const Pose& pose);

        bool ClearedWall(const Pose& pose, double& last_delta);

        const std::vector<WallModel>& GetWalls();

    private:

        void FillAccumulator(const std::vector<double>& ranges, const std::vector<double>& angles);

        void FindPeaks();

        void FindEndPoints(const std::vector<double> &ranges, const std::vector<double> &angles);

        std::vector<std::vector<int>> accumulator_; // Hough transform accumulator for line detection

        std::vector<WallModel> walls_; // stored wall parameters

        const double max_deviation_; // deviation from line model that signifies an endpoint
        const double theta_inc_; // angle resolution in radians
        const std::size_t grid_size_; // size of square accumulator matrix
        const double r_inc_; // range (distance) resolution in meters
        const int detect_threshold_; // min number of Hough votes (scan points) for a valid wall detection

    };

} // turtlebot_maze

#endif //TURTLEBOT_MAZE_WALL_DETECTION_H
