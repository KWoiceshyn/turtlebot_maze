#ifndef TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
#define TURTLEBOT_MAZE_TURTLEBOT_MAZE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>

#include <vector>

class TurtleBotMaze{
public:
    TurtleBotMaze(ros::NodeHandle& nh);
    void init();
    void follow_wall();
    void stop();
    void rotate_angle(float angle_in);
    void callback_laser(const sensor_msgs::LaserScan& msg);
    void update_walls();

private:
    const size_t left_laser_idx_ = 359;
    const size_t right_laser_idx_ = 0;
    const size_t center_laser_idx_= 179;
    const size_t lf_laser_idx_ = 319; //20 deg forward from left laser
    const float k_ = 1.0;
    const float desired_ratio_ = cos(0.349); //20 deg
    float heading_error_;
    float center_range_, left_range_, right_range_;

    std::vector<std::array<float,2>> right_walls_;
    std::vector<std::array<float,2>> left_walls_;

    ros::NodeHandle nh_;
    ros::Publisher vel_publisher_;
    ros::Subscriber laser_subscriber_;
    ros::Rate* loop_rate_;

    ros::ServiceClient teleporter_;
    sensor_msgs::LaserScan current_scan_;
};


#endif //TURTLEBOT_MAZE_TURTLEBOT_MAZE_H
