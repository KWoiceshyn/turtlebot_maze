#include "../include/turtlebot_maze/turtlebot_maze.h"
//#include "pid.h"

#include <csignal>
#include <sstream>
#include <fstream>

namespace turtlebot_maze {

    TurtleBotMaze::TurtleBotMaze(ros::NodeHandle &nh)
            : nh_{nh},
              loop_rate_{new ros::Rate(10)},
              heading_error_{0},
              center_range_{1.0}
    {
        this->init();
        wd_ = std::make_unique<WallDetection>(10.0);
        ph_ = std::make_unique<PositionHistory>();
    }

    TurtleBotMaze::~TurtleBotMaze(){
        wall_detect_record_.close();
    }

    void TurtleBotMaze::init() {
        ROS_INFO("Initializing publishers and subscribers");
        vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
        laser_subscriber_ = nh_.subscribe("/laserscan", 1000, &TurtleBotMaze::callback_laser, this);
        pose_subscriber_ = nh_.subscribe("/odom", 1000, &TurtleBotMaze::callback_pose, this);
        teleporter_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        geometry_msgs::Point init_position;
        init_position.x = -2.0;
        init_position.y = 0.0;
        init_position.z = 0.0;

        geometry_msgs::Quaternion init_orientation;
        init_orientation.x = 0.0;
        init_orientation.y = 0.0;
        init_orientation.z = 0.0;
        init_orientation.w = 1.0;

        geometry_msgs::Pose init_pose;
        init_pose.position = init_position;
        init_pose.orientation = init_orientation;

        gazebo_msgs::ModelState init_model_state;
        init_model_state.model_name = "mobile_base";
        init_model_state.pose = init_pose;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = init_model_state;

        if (teleporter_.call(srv)) {
            ROS_INFO("Moved to start");
        } else {
            ROS_ERROR("Failed move to start:%s", srv.response.status_message.c_str());
        }

        current_pose_.p.x = init_position.x;
        current_pose_.p.y = init_position.y;
        current_pose_.h = 0.0;

        last_wall_update_ = ros::Time::now();
        //TODO: Re-init the hokuyo?
        wall_detect_record_.open("wallDetections.csv");
        if(!wall_detect_record_.is_open())
            std::cerr << "File not open\n";
    }

    void TurtleBotMaze::follow_wall() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.2;
        while (ros::ok()) {
            move_cmd.angular.z = k_ * heading_error_; //simple P-controller
            //ROS_INFO("Ang cmd: %f", move_cmd.angular.z);
            vel_publisher_.publish(move_cmd);
            ros::spinOnce();
            if(ros::Time::now().toSec() - last_wall_update_.toSec() > 1.0 &&
                    current_scan_.ranges[left_laser_idx_] < 1.2 &&
                    current_scan_.ranges[right_laser_idx_] < 1.2){
                update_walls();
                last_wall_update_ = ros::Time::now();
            }
            loop_rate_->sleep();
            if (center_range_ < 0.5) { //too close to wall in front
                stop();
                if (left_range_ < right_range_)
                    rotate_angle(-M_PI_2);
                else
                    rotate_angle(M_PI_2);
            }
        }
    }

    void TurtleBotMaze::stop() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
        vel_publisher_.publish(move_cmd);
    }

    void TurtleBotMaze::rotate_angle(double angle_in) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        if (angle_in > 0)
            move_cmd.angular.z = 0.1;
        else
            move_cmd.angular.z = -0.1;
        double angle_current = 0;
        while (std::abs(angle_current) < std::abs(angle_in)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }
    }

    void TurtleBotMaze::callback_laser(const sensor_msgs::LaserScan &msg) {

        left_range_ = msg.ranges[left_laser_idx_];
        right_range_ = msg.ranges[right_laser_idx_];
        double lf_range = msg.ranges[lf_laser_idx_];
        //when robot is parallel to wall this ratio corresponds to a 20 deg angle
        double actual_ratio = left_range_ / lf_range;
        heading_error_ = desired_ratio_ - actual_ratio; //not actually an angle
        center_range_ = msg.ranges[179];
        //ROS_INFO("Heading error: %f", heading_error_);
        current_scan_ = msg;
    }

    void TurtleBotMaze::callback_pose(const nav_msgs::Odometry &msg) {

        tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        //ROS_INFO("xyzw: %f %f %f %f  %f", q.x(), q.y(), q.z(), q.w(), yaw);
        current_pose_.p.x = msg.pose.pose.position.x;
        current_pose_.p.y = msg.pose.pose.position.y;
        current_pose_.h = yaw;

        current_odom_ = msg;
    }

    void TurtleBotMaze::update_walls() {
        if (current_scan_.header.seq == 0)
            return;

        std::vector<double> angles(hokuyo_num_ranges_, 0.0);
        std::vector<double> ranges(hokuyo_num_ranges_, 0.0);

        for(int i = 0; i < hokuyo_num_ranges_; ++i){
            angles[i] = i * current_scan_.angle_increment;
            ranges[i] = current_scan_.ranges[i];
        }

        wd_->UpdateWalls(current_pose_, ranges, angles);
        ph_->AddPoint(current_pose_.p);
        //ph_->PrintPoints();

        ROS_INFO("Pose: x %f y %f h %f z %f w %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h,
        current_odom_.pose.pose.orientation.z, current_odom_.pose.pose.orientation.w);
        wall_detect_record_ << current_pose_.p.x << "," << current_pose_.p.y << "," << current_pose_.h << ",";
        /*int cc = 0;
        for (const auto &w : wd_->GetWalls()) {
            ROS_INFO("Walls: r %f a %f x0 %f y0 %f x1 %f y1 %f n %d", w.r, w.a, w.p_c.x, w.p_c.y, w.p_e.x, w.p_e.y, ++cc);
        }*/
        auto w = wd_->GetWalls().front();
        ROS_INFO("RWall: r %f a %f x0 %f y0 %f x1 %f y1 %f", w.r, w.a, w.p_c.x, w.p_c.y, w.p_e.x, w.p_e.y);
        wall_detect_record_ << w.p_e.x << "," << w.p_e.y << ",";
        w = wd_->GetWalls().back();
        ROS_INFO("LWall: r %f a %f x0 %f y0 %f x1 %f y1 %f", w.r, w.a, w.p_c.x, w.p_c.y, w.p_e.x, w.p_e.y);
        wall_detect_record_ << w.p_e.x << "," << w.p_e.y;
        //if(cc == 1) wall_detect_record_ << ",";
        wall_detect_record_ << std::endl;
        std::cout << "-----------------------------------\n";

        //write a scan to file
        /*
        static bool has_written = false;
        if (!has_written && center_walls_[0].r <= 3.5){
            std::ofstream ofs;
            ofs.open("sampleScan.txt");
            for (auto i = right_laser_idx_; i <= left_laser_idx_; ++i){
                ofs << i * current_scan_.angle_increment << " " << current_scan_.ranges[i] << std::endl;
            }
            ofs.close();
            has_written = true;
        }*/
    }
}

