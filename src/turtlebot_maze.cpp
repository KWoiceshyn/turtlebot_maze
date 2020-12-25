#include "turtlebot_maze.h"
#include "pid.h"

#include <csignal>
#include <sstream>

TurtleBotMaze::TurtleBotMaze(ros::NodeHandle& nh)
        :nh_(nh),
         loop_rate_(new ros::Rate(10)),
         heading_error_(0),
         center_range_(1.0)
{
    this->init();
}

void TurtleBotMaze::init() {
    ROS_INFO("Initializing publishers and subscribers");
    vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
    laser_subscriber_ = nh_.subscribe("/laserscan", 1000, &TurtleBotMaze::callback_laser, this);
    teleporter_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    geometry_msgs::Point init_position;
    init_position.x = 0.0;
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
    }
    else {
        ROS_ERROR("Failed move to start:%s",srv.response.status_message.c_str());
    }
    //TODO: Re-init the hokuyo?
}

void TurtleBotMaze::follow_wall() {
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.2;
    while (ros::ok()) {
        move_cmd.angular.z = k_*heading_error_; //simple P-controller
        //ROS_INFO("Ang cmd: %f", move_cmd.angular.z);
        vel_publisher_.publish(move_cmd);
        ros::spinOnce();
        update_walls();
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

void TurtleBotMaze::rotate_angle (double angle_in) {
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
        angle_current += move_cmd.angular.z*0.1;
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

void TurtleBotMaze::update_walls() {
    if (current_scan_.header.seq == 0)
        return;

    //simple split of left right and center for now
    update_scan_section(right_laser_idx_, right_walls_);
    update_scan_section(center_start_laser_idx_, center_walls_);
    update_scan_section(center_end_laser_idx_, left_walls_);

    int cc = 0;
    for (const auto& w : right_walls_) {
        ROS_INFO("RW: %f %f %f %f %d", w.r, w.a, w.x, w.y, ++cc);
    }
    cc = 0;
    for (const auto& w : center_walls_) {
        ROS_INFO("CW: %f %f %f %f %d", w.r, w.a, w.x, w.y, ++cc);
    }
    cc = 0;
    for (const auto& w : left_walls_)
        ROS_INFO("LW: %f %f %f %f %d", w.r, w.a, w.x, w.y, ++cc);
    std::cout<<"-----------------------------------\n";
}

void TurtleBotMaze::update_scan_section(const size_t start_idx, std::vector<WallModel>& walls) {
    const double eps = 0.05; // tolerance for new wall group; assumes planar walls for now
    const size_t count_thresh = 30; // min sequential scans to create a wall model
    const size_t pts_to_fit = 5; // number of evenly spaced scan points to fit into wall model
    const size_t section_width = 120; // number of points in 60 deg section
    walls.clear(); // TODO maybe just replace what's there?

    double a_num1 = 0, a_den1 = 0, a_num2 = 0, a_den2 = 0;
    size_t counter = 1;
    for (size_t i = start_idx + 1; i < start_idx + section_width; ++i) {
        double rho = current_scan_.ranges[i];
        if (std::abs(rho - current_scan_.ranges[i-1]) < eps &&
           rho < 0.95 * current_scan_.range_max &&
           i != start_idx + section_width - 1) {
            ++counter;
        }
        else { // new wall group
            if (counter >= count_thresh) {
                
                // compute parameters of current wall and store
                size_t increment = counter / (pts_to_fit - 1);
                if (counter % (pts_to_fit - 1) == 0)
                    --increment;
                
                // pick endpoint in wall group furthest from robot (compare i-counter to i-1)
                size_t longest_idx = current_scan_.ranges[i-counter] >= current_scan_.ranges[i-1] ? i - counter : i - 1;
                double ang = current_scan_.angle_increment * longest_idx;
                double x = current_scan_.ranges[longest_idx] * cos(ang);
                double y = current_scan_.ranges[longest_idx] * sin(ang);
                
                for (size_t j = i - counter; j < i; j += increment) {
                    double rhoj = current_scan_.ranges[j];
                    double thetaj = current_scan_.angle_increment * j; //positive RH angle from section start
                    a_num1 += rhoj * rhoj * sin(2 * thetaj);
                    a_den1 += rhoj * rhoj * cos(2 * thetaj);
                    for (size_t k = i - counter; k < j; k += increment) {
                        double rhok = current_scan_.ranges[k];
                        double thetak = current_scan_.angle_increment * k;
                        a_num2 += rhoj * rhok * sin(thetaj + thetak);
                        a_den2 += rhoj * rhok * cos(thetaj + thetak);
                    }
                }
                
                auto dpts = static_cast<double>(pts_to_fit);
                double a = 0.5 * atan2((1.0 - dpts) * a_num1 / dpts + (2.0 / dpts) * a_num2, (1.0 - dpts) * a_den1 / dpts + (2.0 / dpts) * a_den2);
                double r = 0;
                for (size_t k = i - counter; k < i; k += increment) {
                    r += current_scan_.ranges[k] * cos(current_scan_.angle_increment * k - a);
                }
                r /= dpts;
                if (r < 0){
                    r = -r;
                    a += M_PI;
                }
                walls.push_back({r,a,x,y});
            }
            //reset the accumulators
            a_num1 = 0, a_den1 = 0, a_num2 = 0, a_den2 = 0;
            counter = 1;
        }
    }
}

void mySigintHandler(int sig)
{
    ROS_INFO("Stopping turtlebot");
    ros::shutdown();
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "turtlebot_maze", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    TurtleBotMaze tb_maze(nh);
    tb_maze.follow_wall();

    return 0;
}
