#ifndef TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
#define TURTLEBOT_MAZE_TURTLEBOT_MAZE_H

#include <vector>
#include <fstream>

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

        ~TurtleBotMaze();

        void init();

        void stateMachine();

        void followWall();

        void stop();

        void driveStraight(double distance_in);

        void rotateAngle(double angle_in);

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
        void updateWalls();

        std::vector<int> checkUnvisitedExits(const std::vector<int>& open_exits);

        bool stableEndpointEstimate();

        double stableDesiredHeading();

        double stableDesiredPosition(bool use_x, int error_sign);

        void resetWallEstimates();

        double angleToNearestCompassPoint();

        void callbackLaser(const sensor_msgs::LaserScan &msg);

        void callbackPose(const nav_msgs::Odometry & msg);

        double heading_error_;
        double center_range_, left_range_, right_range_, last_left_range_, last_right_range_;
        double last_min_distance_;

        std::unique_ptr<WallDetection> wd_;
        std::unique_ptr<PositionHistory> ph_;
        std::unique_ptr<PID> pid_;

        Pose current_pose_;
        States current_state_;
        States last_state_;

        std::ofstream wall_detect_record_;

        std::vector<WallModel> wall_estimates_;

        ros::NodeHandle nh_;
        ros::Publisher vel_publisher_;
        ros::Subscriber laser_subscriber_;
        ros::Subscriber pose_subscriber_;

        ros::Time last_wall_update_;
        ros::Rate *loop_rate_;

        ros::ServiceClient teleporter_;
        sensor_msgs::LaserScan current_scan_;
        nav_msgs::Odometry current_odom_;

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
