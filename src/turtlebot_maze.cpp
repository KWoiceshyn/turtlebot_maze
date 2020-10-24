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

void TurtleBotMaze::init(){
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

    if(teleporter_.call(srv)) {
        ROS_INFO("Moved to start");
    }
    else {
        ROS_ERROR("Failed move to start:%s",srv.response.status_message.c_str());
    }
    //TODO: Re-init the hokuyo?
}

void TurtleBotMaze::follow_wall(){
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.2;
    while(ros::ok()) {
        move_cmd.angular.z = k_*heading_error_; //simple P-controller
        //ROS_INFO("Ang cmd: %f", move_cmd.angular.z);
        vel_publisher_.publish(move_cmd);
        ros::spinOnce();
        update_walls();
        loop_rate_->sleep();
        if(center_range_ < 0.5){ //too close to wall in front
            stop();
            if(left_range_ < right_range_)
                rotate_angle(-M_PI_2);
            else
                rotate_angle(M_PI_2);
        }
    }
}

void TurtleBotMaze::stop(){
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
    vel_publisher_.publish(move_cmd);
}

void TurtleBotMaze::rotate_angle(float angle_in) {
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.0;
    if(angle_in > 0)
        move_cmd.angular.z = 0.1;
    else
        move_cmd.angular.z = -0.1;
    float angle_current = 0;
    while(std::abs(angle_current) < std::abs(angle_in)){
        vel_publisher_.publish(move_cmd);
        loop_rate_->sleep();
        angle_current += move_cmd.angular.z*0.1;
        //ROS_INFO("Rotating: %f", angle_current);
    }
}

void TurtleBotMaze::callback_laser(const sensor_msgs::LaserScan &msg) {
    left_range_ = msg.ranges[left_laser_idx_];
    right_range_ = msg.ranges[right_laser_idx_];
    float lf_range = msg.ranges[lf_laser_idx_];
    //when robot is parallel to wall this ratio corresponds to a 20 deg angle
    float actual_ratio = left_range_ / lf_range;
    heading_error_ = desired_ratio_ - actual_ratio; //not actually an angle
    center_range_ = msg.ranges[center_laser_idx_];
    //ROS_INFO("Heading error: %f", heading_error_);
    current_scan_ = msg;
}

void TurtleBotMaze::update_walls(){
    if(current_scan_.header.seq == 0)
        return;
    const float eps = 0.1;
    const size_t count_thresh = 10;
    right_walls_.clear();
    left_walls_.clear();

    //simple split of left and right sides for now
    float rhoi = current_scan_.ranges[0];
    float thetai = 0; //angle from where?
    float a_num1 = rhoi*rhoi*sin(2*thetai);
    float a_den1 = rhoi*rhoi*cos(2*thetai);
    float a_num2 = 0, a_den2 = 0;
    size_t counter = 1;
    // TODO dont use ranges that are within say 95% of max
    for(size_t i = 1; i < center_laser_idx_; ++i){
        rhoi = current_scan_.ranges[i];
        thetai = current_scan_.angle_increment*i; //angle from where?
        if(std::abs(rhoi - current_scan_.ranges[i-1]) < eps){
            a_num1 += rhoi*rhoi*sin(2*thetai);
            a_den1 += rhoi*rhoi*cos(2*thetai);
            ++counter;
        }else{ // new wall group
            if(counter >= count_thresh){
                //compute parameters of current wall and store
                for(size_t j = i - counter; j < i; ++j){
                    float rhoj = current_scan_.ranges[j];
                    float thetaj = current_scan_.angle_increment*j;
                    for(size_t k = i - counter; k < i; ++k){
                        float rhok = current_scan_.ranges[k];
                        float thetak = current_scan_.angle_increment*k;
                        a_num2 += rhoj*rhok*cos(thetaj)*sin(thetak);
                        a_den2 += rhoj*rhok*cos(thetaj + thetak);
                    }
                }
                float a = 0.5*atan((a_num1 - (2.0/counter)*a_num2)/(a_den1 - (1.0/counter)*a_den2));
                float r = 0;
                for(size_t k = i - counter; k < i; ++k){
                    r += current_scan_.ranges[k]*cos(current_scan_.angle_increment*k - a);
                }
                r /= counter;
                right_walls_.push_back({r,a});
                //ROS_INFO("Wall: %f, %f", r, a);
            }
            //reset the accumulators
            a_num1 = rhoi*rhoi*sin(2*thetai);
            a_den1 = rhoi*rhoi*cos(2*thetai);
            a_num2 = 0, a_den2 = 0;
            counter = 1;
        }//TODO collect last wall segment
    }
    if(counter >= count_thresh){
        //compute parameters of current wall and store
        for(size_t j = center_laser_idx_ - counter; j < center_laser_idx_; ++j){
            float rhoj = current_scan_.ranges[j];
            float thetaj = current_scan_.angle_increment*j;
            for(size_t k = center_laser_idx_ - counter; k < center_laser_idx_; ++k){
                float rhok = current_scan_.ranges[k];
                float thetak = current_scan_.angle_increment*k;
                a_num2 += rhoj*rhok*cos(thetaj)*sin(thetak);
                a_den2 += rhoj*rhok*cos(thetaj + thetak);
            }
        }
        float a = 0.5*atan((a_num1 - (2.0/counter)*a_num2)/(a_den1 - (1.0/counter)*a_den2));
        a += M_PI_2; //TODO: how to get front wall the right orientation?
        float r = 0;
        for(size_t k = center_laser_idx_ - counter; k < center_laser_idx_; ++k){
            r += current_scan_.ranges[k]*cos(current_scan_.angle_increment*k - a);
        }
        r /= counter;
        right_walls_.push_back({r,a});
        //ROS_INFO("Wall: %f, %f", r, a);
    }

    //loop through left side points
    rhoi = current_scan_.ranges[left_laser_idx_];
    thetai = 0; //angle from where?
    a_num1 = rhoi*rhoi*sin(2*thetai);
    a_den1 = rhoi*rhoi*cos(2*thetai);
    a_num2 = 0, a_den2 = 0;
    counter = 1;
    // TODO dont use ranges that are within say 95% of max
    for(size_t i = left_laser_idx_-1; i >= center_laser_idx_; --i){
        rhoi = current_scan_.ranges[i];
        thetai = current_scan_.angle_increment*(left_laser_idx_ - i); //angle from where?
        if(std::abs(rhoi - current_scan_.ranges[i+1]) < eps){
            a_num1 += rhoi*rhoi*sin(2*thetai);
            a_den1 += rhoi*rhoi*cos(2*thetai);
            ++counter;
        }else{ // new wall group
            if(counter >= count_thresh){
                //compute parameters of current wall and store
                for(size_t j = i+1; j <= i + counter; ++j){
                    float rhoj = current_scan_.ranges[j];
                    float thetaj = current_scan_.angle_increment*(left_laser_idx_ - j);
                    for(size_t k = i+1; k <= i + counter; ++k){
                        float rhok = current_scan_.ranges[k];
                        float thetak = current_scan_.angle_increment*(left_laser_idx_ - k);
                        a_num2 += rhoj*rhok*cos(thetaj)*sin(thetak);
                        a_den2 += rhoj*rhok*cos(thetaj + thetak);
                    }
                }
                float a = 0.5*atan((a_num1 - (2.0/counter)*a_num2)/(a_den1 - (1.0/counter)*a_den2));
                float r = 0;
                for(size_t k = i+1; k <= i + counter; ++k){
                    r += current_scan_.ranges[k]*cos(current_scan_.angle_increment*(left_laser_idx_ - k) - a);
                }
                r /= counter;
                left_walls_.push_back({r,a});
                //ROS_INFO("Wall: %f, %f", r, a);
            }
            //reset the accumulators
            a_num1 = rhoi*rhoi*sin(2*thetai);
            a_den1 = rhoi*rhoi*cos(2*thetai);
            a_num2 = 0, a_den2 = 0;
            counter = 1;
        }
    }
    int cc = 0;
    for(const auto& v : right_walls_){
        ROS_INFO("RW: %f %f %d",v[0],v[1],++cc);
    }
    cc = 0;
    for(const auto& v : left_walls_)
        ROS_INFO("LW: %f %f %d",v[0],v[1],++cc);


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
