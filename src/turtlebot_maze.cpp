#include "../include/turtlebot_maze/turtlebot_maze.h"
//#include "pid.h"

#include <csignal>

namespace turtlebot_maze {

    TurtleBotMaze::TurtleBotMaze(ros::NodeHandle &nh)
            : nh_{nh},
              loop_rate_{new ros::Rate(10)},
              heading_error_{0},
              center_range_{1.0},
              current_state_{States::INITIALIZE},
              last_state_{States::INITIALIZE},
              last_min_distance_{1e3}
    {
        init();
        wd_ = std::make_unique<WallDetection>(4.0);
        ph_ = std::make_unique<PositionHistory>();
        pid_ = std::make_unique<PID>(0.5, 0.05, 1.0);
        wall_estimates_ = {WallModel(), WallModel()};
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

    void TurtleBotMaze::state_machine() {

        switch(current_state_){

            case States::INITIALIZE: {
                //ros::spinOnce(); // get callbacks
                update_walls();
                // TODO: make below checks based on wall model
                if (left_range_ < corridor_max_wall_dist_ &&
                    right_range_ < corridor_max_wall_dist_) {
                    ROS_INFO("Initial state: corridor");
                    current_state_ = States::CORRIDOR;
                } else {
                    ROS_INFO("Initial state: intersection");
                    current_state_ = States::INTERSECTION;
                }
                break;
            }

            case States::CORRIDOR:{
                if(last_state_ != current_state_){
                    ROS_INFO("Entered Corridor state");
                    last_state_ = current_state_;
                    pid_->Reset(ros::Time::now().toSec());
                    update_walls();
                    last_wall_update_ = ros::Time::now();
                }
                geometry_msgs::Twist move_cmd;
                move_cmd.linear.x = 0.2;
                //move_cmd.angular.z = k_ * heading_error_; //simple P-controller
                heading_error_ = AngleDifference(stable_desired_heading(), current_pose_.h);
                if(std::fabs(heading_error_) > M_PI_4){
                    ROS_INFO("HEADING ERROR %f", heading_error_);
                    heading_error_ = 0;
                }
                // robot traveling in positive x direction
                double test_dp = stable_desired_position();
                double pos_fb = current_pose_.p.y;
                double error_sign =1.0;
                if(std::fabs(std::fabs(current_pose_.h) - M_PI) < M_PI_4){ // robot traveling in negative x direction
                    error_sign = -1.0;
                    pos_fb = current_pose_.p.y;
                    //ROS_INFO("POSITION ERROR -X %f %f", test_dp, pos_fb);
                }
                if(std::fabs(current_pose_.h - M_PI_2) < M_PI_4){ // robot traveling in positive y direction
                    error_sign = -1.0;
                    pos_fb = current_pose_.p.x;
                    //ROS_INFO("POSITION ERROR +Y %f %f", test_dp, pos_fb);
                }
                if(std::fabs(current_pose_.h + M_PI_2) < M_PI_4){ // robot traveling in negative y direction
                    error_sign = 1.0;
                    pos_fb = current_pose_.p.x;
                    //ROS_INFO("POSITION ERROR -Y %f %f", test_dp, pos_fb);
                }
                if(std::fabs(test_dp) > 1e-3) {
                    //ROS_INFO("PE %f %f", test_dp, pos_fb);
                    move_cmd.angular.z = pid_->RunControlHE(error_sign*(test_dp - pos_fb), ros::Time::now().toSec(), heading_error_);
                }
                else
                    move_cmd.angular.z = pid_->RunControlHE(0.0, ros::Time::now().toSec(), heading_error_);

                // TODO: Use the wd_ and PID controller
                //ROS_INFO("Ang cmd: %f", move_cmd.angular.z);
                vel_publisher_.publish(move_cmd);
                if(ros::Time::now().toSec() - last_wall_update_.toSec() > 1.0){
                    update_walls(); // TODO: call the wd_ and ph_ directly here
                    if (stable_endpoint_estimate()){
                        ROS_INFO("STABLE ENDPOINT ESTIMATE");
                        double min_distance = std::min(Distance(current_pose_.p, wall_estimates_[0].p_e), Distance(current_pose_.p, wall_estimates_[1].p_e));
                        if(min_distance - last_min_distance_ > 1e-3){
                            ROS_INFO("CLEARED WALL %f %f", min_distance, last_min_distance_);
                            stop();
                            current_state_ = States::INTERSECTION;
                            pid_->Reset(ros::Time::now().toSec());
                            wd_->ResetMedians();
                            reset_wall_estimates();
                            stable_desired_heading(); // clear the static variables
                            stable_desired_position();
                        }
                        last_min_distance_ = min_distance;
                    }
                    last_wall_update_ = ros::Time::now();
                }
                /*if(left_range_ > 1.2 * corridor_max_wall_dist_ ||
                   center_range_ < corridor_max_wall_dist_ ||
                   right_range_ > 1.2 * corridor_max_wall_dist_){
                    // TODO: use the ClearedWall() from wd_
                    current_state_ = States::INTERSECTION;
                    stop();
                }*/
            }
                break;

            case States::INTERSECTION: {
                if (last_state_ != current_state_) {
                    ROS_INFO("Entered Intersection state");
                    last_state_ = current_state_;
                    last_min_distance_ = 1e3;
                }
                drive_straight(0.5); // drive forward a bit to clear walls
                 // TODO: control this drive straight to the last wall
                ros::spinOnce();
                //update_walls();
                // check exits
                std::vector<int> open_exits = check_open_exits();

                double angle_to_rotate = 0.0;
                if (std::any_of(open_exits.begin(), open_exits.end(), [](int a) { return a != 0; })) {
                    // draw randomly
                    int draw = rand() % 3;
                    while (open_exits[draw] == 0)
                        draw = rand() % 3;
                    if (draw == 0){
                        angle_to_rotate = M_PI_2;
                        ROS_INFO("Chose left");
                    } else if (draw == 2){
                        angle_to_rotate = -M_PI_2;
                        ROS_INFO("Chose right");
                    }else{
                        ROS_INFO("Chose center");
                    }

                } else {
                    // dead end, go back the way you came
                    angle_to_rotate = M_PI;
                }

                rotate_angle(angle_to_rotate);
                drive_straight(1.0);
                current_state_ = States::CORRIDOR;
                break;
            }

            case States::ESCAPED:{
                ROS_INFO("Robot escaped maze!");
                stop();
                break;
            }
        }
    }

    void TurtleBotMaze::follow_wall() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.2;
        //while(current_scan_.header.seq == 0)
            //ros::spinOnce();
        //update_walls();
        //last_wall_update_ = ros::Time::now();
        while (ros::ok()) {
            //move_cmd.angular.z = k_ * heading_error_; //simple P-controller
            //heading_error_ = wall_estimates_[1].a - (current_pose_.h + M_PI_2);
            move_cmd.angular.z = pid_->RunControlHE(0.6 - left_range_, ros::Time::now().toSec(), heading_error_);
            //ROS_INFO("LR: %f HE: %f", left_range_, heading_error_);
            vel_publisher_.publish(move_cmd);
            ros::spinOnce();
            if(ros::Time::now().toSec() - last_wall_update_.toSec() > 1.0){
                if (left_range_ < 1.2 && right_range_ < 1.2){
                    update_walls();
                    if (stable_endpoint_estimate()){
                        ROS_INFO("STABLE ENDPOINT ESTIMATE");
                        double min_distance = std::min(Distance(current_pose_.p, wall_estimates_[0].p_e), Distance(current_pose_.p, wall_estimates_[1].p_e));
                        if(min_distance - last_min_distance_ > 1e-3){
                            ROS_INFO("CLEARED WALL %f %f", min_distance, last_min_distance_);
                        }
                        last_min_distance_ = min_distance;
                    }

                }else{
                    last_min_distance_ = 1e3;
                    wd_->ResetMedians();
                    reset_wall_estimates();
                }
                last_wall_update_ = ros::Time::now();
            }
            loop_rate_->sleep();
            if (center_range_ < 0.5) { //too close to wall in front
                stop();
                if (left_range_ < right_range_)
                    rotate_angle(-M_PI_2);
                else
                    rotate_angle(M_PI_2);
                pid_->Reset(ros::Time::now().toSec());
            }
        }
    }

    void TurtleBotMaze::stop() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
        vel_publisher_.publish(move_cmd);
    }

    void TurtleBotMaze::drive_straight(double distance_in) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = distance_in >= 0 ? 0.2 : -0.2;
        move_cmd.angular.z = 0.0;
        double distance_current = 0;
        while (std::fabs(distance_current) < std::fabs(distance_in)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            distance_current += move_cmd.linear.x * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }

    }

    void TurtleBotMaze::rotate_angle(double angle_in) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = angle_in >= 0 ? 0.5 : - 0.5;
        double angle_current = 0;
        while (std::fabs(angle_current) < std::fabs(angle_in)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }
    }

    void TurtleBotMaze::callback_laser(const sensor_msgs::LaserScan &msg) {

        left_range_ = msg.ranges[left_laser_idx_];
        center_range_ = msg.ranges[center_laser_idx_];
        right_range_ = msg.ranges[right_laser_idx_];
        //double lf_range = msg.ranges[lf_laser_idx_];
        //when robot is parallel to wall this ratio corresponds to a 20 deg angle
        //double actual_ratio = left_range_ / lf_range;
        //heading_error_ = desired_ratio_ - actual_ratio; //not actually an angle
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

        ROS_INFO("Pose: x %f y %f h %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h);
        wall_detect_record_ << current_pose_.p.x << "," << current_pose_.p.y << "," << current_pose_.h << ",";
        /*int cc = 0;
        for (const auto &w : wd_->GetWalls()) {
            ROS_INFO("Walls: r %f a %f x0 %f y0 %f x1 %f y1 %f n %d", w.r, w.a, w.p_c.x, w.p_c.y, w.p_e.x, w.p_e.y, ++cc);
        }*/
        wd_->GetWalls(wall_estimates_[1], wall_estimates_[0]);
        ROS_INFO("RWall: r %f a %f x0 %f y0 %f x1 %f y1 %f", wall_estimates_[0].r, wall_estimates_[0].a, wall_estimates_[0].p_c.x, wall_estimates_[0].p_c.y,
                 wall_estimates_[0].p_e.x, wall_estimates_[0].p_e.y);
        wall_detect_record_ << wall_estimates_[0].p_e.x << "," << wall_estimates_[0].p_e.y << ",";
        ROS_INFO("LWall: r %f a %f x0 %f y0 %f x1 %f y1 %f", wall_estimates_[1].r, wall_estimates_[1].a, wall_estimates_[1].p_c.x, wall_estimates_[1].p_c.y,
                 wall_estimates_[1].p_e.x, wall_estimates_[1].p_e.y);
        wall_detect_record_ << wall_estimates_[1].p_e.x << "," << wall_estimates_[1].p_e.y;
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

    std::vector<int> TurtleBotMaze::check_open_exits() {
        std::vector<int> open_exits(3);
        // TODO: perhaps use a median of adjacent ranges rather than just one
        open_exits[0] = left_range_ > corridor_max_wall_dist_ ? 1 : 0;
        open_exits[1] = center_range_ > 2.0 * corridor_max_wall_dist_ ? 1 : 0;
        open_exits[2] = right_range_ > corridor_max_wall_dist_ ? 1 : 0;
        ROS_INFO("Open exits: L %d C %d R %d", open_exits[0], open_exits[1], open_exits[2]);
        // TODO: how to check for escaped state?

        // check if open exits already visited
        ROS_INFO("Pose: x %f y %f h %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h);
        if (open_exits[0]) {
            Point ll = TransformToGlobal({-1.0, 2.5}, current_pose_);
            Point ur = TransformToGlobal({1.0, 0.5}, current_pose_);
            if (ph_->AnyInRectangle(ll, ur)) {
                open_exits[0] = 0;
                ROS_INFO("Left exit already visited");
            }
        }
        if (open_exits[1]) {
            Point ll = TransformToGlobal({0.5, 1.0}, current_pose_);
            Point ur = TransformToGlobal({2.5, -1.0}, current_pose_);
            if (ph_->AnyInRectangle(ll, ur)) {
                open_exits[1] = 0;
                ROS_INFO("Center exit already visited");
            }
        }
        if (open_exits[2]) {
            Point ll = TransformToGlobal({-1.0, -0.5}, current_pose_);
            Point ur = TransformToGlobal({1.0, -2.5}, current_pose_);
            if (ph_->AnyInRectangle(ll, ur)) {
                open_exits[2] = 0;
                ROS_INFO("Right exit already visited");
            }
        }
        return open_exits;
    }

    bool TurtleBotMaze::stable_endpoint_estimate() {
        static int stable_count = 0;
        static Point last_right_point, last_left_point;
        if(std::fabs(wall_estimates_[0].p_e.x) > 1e-3 && std::fabs(wall_estimates_[1].p_e.x) > 1e-3){
            if(Distance(last_right_point, wall_estimates_[0].p_e) < 0.1 && Distance(last_left_point, wall_estimates_[1].p_e) < 0.1)
                ++stable_count;
        }else{
            stable_count = 0;
        }
        last_right_point = wall_estimates_[0].p_e;
        last_left_point = wall_estimates_[1].p_e;
        return stable_count >= 2;
    }

    double TurtleBotMaze::stable_desired_heading() {
        static int stable_count = 0;
        static double last_left_wall_est = M_PI_4;
        if(wall_estimates_[0].r > 0.1 && wall_estimates_[1].r > 0.1){
            if(stable_count < 3 && AngleDifference(last_left_wall_est, wall_estimates_[1].a) < 0.1){
                ++stable_count;
            }
            if(stable_count == 3){
                //ROS_INFO("STABLE DESIRED HEADING %f", last_left_wall_est);
                return last_left_wall_est - M_PI_2;
            }

            last_left_wall_est = wall_estimates_[1].a;
        }else{
            last_left_wall_est = M_PI_4;
            stable_count = 0;
        }
        return current_pose_.h;
    }

    double TurtleBotMaze::stable_desired_position(){
        static int stable_count = 0;
        static Point last_right_point, last_left_point;
        if(std::fabs(wall_estimates_[0].p_c.x) > 1e-3 && std::fabs(wall_estimates_[1].p_c.x) > 1e-3){
            if(std::fabs(std::fabs(current_pose_.h) - M_PI_2) < M_PI_4){ // robot facing along global y axis, care about x-coordinate
                if(stable_count < 3 && std::fabs(last_right_point.x - wall_estimates_[0].p_c.x) < 0.1 &&
                        std::fabs(last_left_point.x - wall_estimates_[1].p_c.x) < 0.1){
                    ++stable_count;
                }
                if(stable_count == 3){
                    //ROS_INFO("STABLE DESIRED POSITION Y %f", (last_left_point.x + last_right_point.x) / 2);
                    return (last_left_point.x + last_right_point.x) / 2;
                }
            } else{ // robot facing along global x-axis, care about y-coordinate
                if(stable_count < 3 && std::fabs(last_right_point.y - wall_estimates_[0].p_c.y) < 0.1 &&
                   std::fabs(last_left_point.y - wall_estimates_[1].p_c.y) < 0.1){
                    ++stable_count;
                }
                if(stable_count == 3){
                    //ROS_INFO("STABLE DESIRED POSITION X %f", (last_left_point.y + last_right_point.y) / 2);
                    return (last_left_point.y + last_right_point.y) / 2;
                }
            }

            last_right_point = wall_estimates_[0].p_c;
            last_left_point = wall_estimates_[1].p_c;
        }else{
            stable_count = 0;
            last_right_point = Point();
            last_left_point = Point();
        }
        return 0.0;
    }

    void TurtleBotMaze::reset_wall_estimates() {
        wall_estimates_[0].r = 0;
        wall_estimates_[0].a = 0;
        wall_estimates_[0].p_c = Point();
        wall_estimates_[0].p_e = Point();
        wall_estimates_[1].r = 0;
        wall_estimates_[1].a = 0;
        wall_estimates_[1].p_c = Point();
        wall_estimates_[1].p_e = Point();
    }

} // turtlebot_maze

