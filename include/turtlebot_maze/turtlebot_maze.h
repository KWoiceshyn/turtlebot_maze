#ifndef TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
#define TURTLEBOT_MAZE_TURTLEBOT_MAZE_H

#include "wall_detection.h"
#include "position_history.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

#include <vector>

namespace turtlebot_maze {

    class TurtleBotMaze {
    public:
        TurtleBotMaze(ros::NodeHandle &nh);

        void init();

        void follow_wall();

        void stop();

        void rotate_angle(double angle_in);

        void callback_laser(const sensor_msgs::LaserScan &msg);

        void callback_pose(const nav_msgs::Odometry & msg);

    private:


        void update_walls();

        const size_t right_laser_idx_ = 0;
        const size_t center_start_laser_idx_ = 120;
        const size_t center_end_laser_idx_ = 240;
        const size_t lf_laser_idx_ = 320; //20 deg forward from left laser
        const size_t left_laser_idx_ = 359;
        const size_t hokuyo_num_ranges_ = 360;
        const double k_ = 1.0;
        const double desired_ratio_ = cos(0.349); //20 deg
        double heading_error_;
        double center_range_, left_range_, right_range_;

        std::unique_ptr<WallDetection> wd_;
        std::unique_ptr<PositionHistory> ph_;

        Pose current_pose_;

        //std::vector<WallModel> right_walls_;
        //std::vector<WallModel> center_walls_;
        //std::vector<WallModel> left_walls_;

        ros::NodeHandle nh_;
        ros::Publisher vel_publisher_;
        ros::Subscriber laser_subscriber_;
        ros::Subscriber pose_subscriber_;
        ros::Rate *loop_rate_;

        ros::Time last_wall_update_;

        ros::ServiceClient teleporter_;
        sensor_msgs::LaserScan current_scan_;
    };

} // turtlebot_maze


#endif //TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
