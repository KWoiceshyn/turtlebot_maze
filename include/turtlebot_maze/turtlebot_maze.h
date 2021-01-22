#ifndef TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
#define TURTLEBOT_MAZE_TURTLEBOT_MAZE_H

#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

#include "turtlebot_maze/wall_detection.h"
#include "turtlebot_maze/position_history.h"
#include "turtlebot_maze/pid.h"

namespace turtlebot_maze {

    class TurtleBotMaze {
    public:
        explicit TurtleBotMaze(ros::NodeHandle &nh);

        void init();

        void stateMachine();

        //void followWall();

        void stop();

        void driveStraight(double distance);

        void rotateAngle(double angle);

        void sleep(){
            loop_rate_->sleep();
        }

        enum class States{
            INITIALIZE,
            CORRIDOR,
            INTERSECTION,
            ESCAPED
        };

        States getState() const{
            return last_state_;
        }

    private:

        // get updated left and right wall models from the wall_detection object
        void getUpdatedWalls();

        std::vector<int> checkUnvisitedExits(const std::vector<int>& open_exits);

        // checks if both left wall and right wall endpoint estimates are consistent
        bool stableEndpointEstimate();

        // return orientation of wall model if it is reliable, so that the robot can drive parallel to it
        // if it is not reliable, return the current heading of the robot so that the controller ignores heading error
        double stableDesiredHeading();

        // depending on robot's heading, calculate the robot's error w.r.t the desired position perpendicular to the direction of travel
        // this desired position is either the center-line of a corridor (when left and right walls are close)
        // or a short distance from a wall if only one wall is present; returns zero if wall estimates are not reliable
        double stableDesiredPosition(const Pose& pose);

        // clear out the local left and right wall estimates
        void resetWallEstimates();

        enum class Directions{
            POS_X,
            NEG_X,
            POS_Y,
            NEG_Y
        };

        // find the nearest cardinal direction to the robot's current heading
        Directions nearestCompassPoint(double heading);

        // rotate to align with nearest cardinal direction
        void rotateToNearestCompassPoint(double heading);

        void callbackLaser(const sensor_msgs::LaserScan &msg);

        void callbackPose(const nav_msgs::Odometry & msg);

        double center_range_, left_range_, right_range_;

        std::unique_ptr<WallDetection> wd_;
        std::unique_ptr<PositionHistory> ph_;
        std::unique_ptr<PID> pid_;

        Pose current_pose_;
        States current_state_;
        States last_state_;

        std::vector<WallModel> wall_estimates_; // local left and right wall model estimates while in a corridor

        ros::NodeHandle nh_;
        ros::Publisher vel_publisher_;
        ros::Subscriber laser_subscriber_;
        ros::Subscriber pose_subscriber_;

        ros::Time last_wall_update_;
        ros::Rate *loop_rate_;

        ros::ServiceClient teleporter_;
        sensor_msgs::LaserScan current_scan_;

        const size_t right_laser_idx_ {0};
        const size_t center_laser_idx_ {179};
        const size_t lf_laser_idx_ {320}; //20 deg forward from left laser
        const size_t left_laser_idx_ {359};
        const size_t hokuyo_num_ranges_ {360};
        const double desired_ratio_ {cos(0.349)}; //20 deg
        const double corridor_max_wall_dist_ {1.0}; // max lateral distance to wall on either side to be in a corridor
    };

} // namespace turtlebot_maze


#endif //TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
