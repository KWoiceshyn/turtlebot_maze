#ifndef TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
#define TURTLEBOT_MAZE_TURTLEBOT_MAZE_H

#include "wall_detection.h"
#include "position_history.h"
#include "pid.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <fstream>

namespace turtlebot_maze {

    class TurtleBotMaze {
    public:
        explicit TurtleBotMaze(ros::NodeHandle &nh);

        ~TurtleBotMaze();

        void init();

        void state_machine();

        void follow_wall();

        void stop();

        void drive_straight(double distance_in);

        void rotate_angle(double angle_in);

        void callback_laser(const sensor_msgs::LaserScan &msg);

        void callback_pose(const nav_msgs::Odometry & msg);

        ros::Rate *loop_rate_;

    private:

        enum class States{
            INITIALIZE,
            CORRIDOR,
            INTERSECTION,
            ESCAPED
        };

        void update_walls();

        std::vector<int> check_open_exits();

        bool stable_endpoint_estimate();

        double stable_desired_heading();

        double stable_desired_position(bool use_x, int error_sign);

        void reset_wall_estimates();

        double angle_to_nearest_compass_point();

        const size_t right_laser_idx_ = 0;
        const size_t center_laser_idx_ = 179;
        const size_t lf_laser_idx_ = 320; //20 deg forward from left laser
        const size_t left_laser_idx_ = 359;
        const size_t hokuyo_num_ranges_ = 360;
        //const double k_ = 1.0;
        const double desired_ratio_ = cos(0.349); //20 deg
        const double corridor_max_wall_dist_ {1.0}; // max lateral distance to wall on either side to be in a corridor
        double heading_error_;
        double center_range_, left_range_, right_range_;
        double last_min_distance_;

        std::unique_ptr<WallDetection> wd_;
        std::unique_ptr<PositionHistory> ph_;
        std::unique_ptr<PID> pid_;

        Pose current_pose_;
        States current_state_;
        States last_state_;

        std::ofstream wall_detect_record_;

        std::vector<WallModel> wall_estimates_;
        //std::vector<WallModel> center_walls_;
        //std::vector<WallModel> left_walls_;

        ros::NodeHandle nh_;
        ros::Publisher vel_publisher_;
        ros::Subscriber laser_subscriber_;
        ros::Subscriber pose_subscriber_;

        ros::Time last_wall_update_;

        ros::ServiceClient teleporter_;
        sensor_msgs::LaserScan current_scan_;
        nav_msgs::Odometry current_odom_;
    };

} // turtlebot_maze


#endif //TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
