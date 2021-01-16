#include "turtlebot_maze/turtlebot_maze.h"

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
        laser_subscriber_ = nh_.subscribe("/laserscan", 1000, &TurtleBotMaze::callbackLaser, this);
        pose_subscriber_ = nh_.subscribe("/odom", 1000, &TurtleBotMaze::callbackPose, this);
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

    void TurtleBotMaze::stateMachine() {

        switch(current_state_){

            case States::INITIALIZE: {
                //ros::spinOnce(); // get callbacks
                updateWalls();
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
                    pid_->reset(ros::Time::now().toSec());
                    updateWalls();
                    last_wall_update_ = ros::Time::now(); // TODO: put this in the update_walls function!!!
                    last_left_range_ = left_range_;
                    last_right_range_ = right_range_;
                }
                geometry_msgs::Twist move_cmd;
                move_cmd.linear.x = 0.2;
                //move_cmd.angular.z = k_ * heading_error_; //simple P-controller
                heading_error_ = angleDifference(stableDesiredHeading(), current_pose_.h);
                if(std::fabs(heading_error_) > M_PI_4){
                    ROS_INFO("HEADING ERROR %f", heading_error_);
                    heading_error_ = 0;
                }

                double pos_fb = 0, test_dp = 0;
                int error_sign = 0; // TODO: make error wrt any line rather than 4 cardinal directions
                if(std::fabs(current_pose_.h) < M_PI_4){ // robot traveling in positive x direction
                    error_sign = 1;
                    test_dp = stableDesiredPosition(true, error_sign);
                    pos_fb = current_pose_.p.y;
                }
                else if(std::fabs(std::fabs(current_pose_.h) - M_PI) < M_PI_4){ // robot traveling in negative x direction
                    error_sign = -1;
                    test_dp = stableDesiredPosition(true, error_sign);
                    pos_fb = current_pose_.p.y;
                    //ROS_INFO("POSITION ERROR -X %f %f", test_dp, pos_fb);
                }
                else if(std::fabs(current_pose_.h - M_PI_2) < M_PI_4){ // robot traveling in positive y direction
                    error_sign = -1;
                    test_dp = stableDesiredPosition(false, error_sign);
                    pos_fb = current_pose_.p.x;
                    //ROS_INFO("POSITION ERROR +Y %f %f", test_dp, pos_fb);
                }
                else if(std::fabs(current_pose_.h + M_PI_2) < M_PI_4){ // robot traveling in negative y direction
                    error_sign = 1;
                    test_dp = stableDesiredPosition(false, error_sign);
                    pos_fb = current_pose_.p.x;
                    //ROS_INFO("POSITION ERROR -Y %f %f", test_dp, pos_fb);
                }
                if(std::fabs(test_dp) > 1e-3) {
                    //ROS_INFO("PE %f %f", test_dp, pos_fb);
                    move_cmd.angular.z = pid_->runControlHE(error_sign*(test_dp - pos_fb), ros::Time::now().toSec(), heading_error_);
                }
                else
                    move_cmd.angular.z = pid_->runControlHE(0.0, ros::Time::now().toSec(), heading_error_);

                //ROS_INFO("Ang cmd: %f", move_cmd.angular.z);
                vel_publisher_.publish(move_cmd);
                if(ros::Time::now().toSec() - last_wall_update_.toSec() > 0.2){
                    updateWalls(); // TODO: call the wd_ and ph_ directly here
                    if (stableEndpointEstimate()){
                        ROS_INFO("STABLE ENDPOINT ESTIMATE");
                        double min_distance = std::min(distance(current_pose_.p, wall_estimates_[0].p_e), distance(current_pose_.p, wall_estimates_[1].p_e));
                        if(min_distance - last_min_distance_ > 1e-3){
                            ROS_INFO("CLEARED WALL %f %f", min_distance, last_min_distance_);
                            stop();
                            current_state_ = States::INTERSECTION;
                            pid_->reset(ros::Time::now().toSec());
                            wd_->resetMedians();
                            resetWallEstimates();
                            stableDesiredHeading(); // clear the static variables
                            stableDesiredPosition(false, 0);
                        }
                        last_min_distance_ = min_distance;
                    }
                    last_wall_update_ = ros::Time::now();
                }

                if(center_range_ < 0.8 || left_range_ - last_left_range_ > 1.0 || right_range_ - last_right_range_ > 1.0){
                    stop();
                    current_state_ = States::INTERSECTION;
                    pid_->reset(ros::Time::now().toSec());
                    wd_->resetMedians();
                    resetWallEstimates();
                    stableDesiredHeading(); // clear the static variables
                    stableDesiredPosition(false, 0);
                }
                last_left_range_ = left_range_;
                last_right_range_ = right_range_;
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
                if(center_range_ > 1.0)
                    driveStraight(0.5); // drive forward a bit to clear walls
                 // TODO: control this drive straight to the last wall
                ros::spinOnce();
                //update_walls();

                // check exits
                std::vector<int> open_exits(3);
                // TODO: perhaps use a median of adjacent ranges rather than just one
                open_exits[0] = left_range_ > 1.5 * corridor_max_wall_dist_ ? 1 : 0;
                open_exits[1] = center_range_ > 2.0 * corridor_max_wall_dist_ ? 1 : 0;
                open_exits[2] = right_range_ > 1.5 * corridor_max_wall_dist_ ? 1 : 0;
                ROS_INFO("Open exits: L %d C %d R %d", open_exits[0], open_exits[1], open_exits[2]);

                double angle_to_rotate = 0.0;
                if (std::any_of(open_exits.begin(), open_exits.end(), [](int a) { return a != 0; })) {

                    std::vector<int> unvisited_exits = checkUnvisitedExits(open_exits);
                    ROS_INFO("Unvisited exits: L %d C %d R %d", unvisited_exits[0], unvisited_exits[1], unvisited_exits[2]);
                    if(std::none_of(unvisited_exits.begin(), unvisited_exits.end(), [](int a) { return a == 1; })){
                        // if all are already visited, choose left, center, right based on openness
                        if(open_exits[0] == 1){
                            angle_to_rotate = M_PI_2;
                            ROS_INFO("Chose left already visited");
                        }
                        else if(open_exits[1] == 1){
                            angle_to_rotate = 0;
                            ROS_INFO("Chose center already visited");
                        }
                        else if(open_exits[2] == 1){
                            angle_to_rotate = -M_PI_2;
                            ROS_INFO("Chose right already visited");
                        }
                    }else{
                        // choose center first, then left, then right
                        if(unvisited_exits[1] == 1){
                            ROS_INFO("Chose center");
                            angle_to_rotate = 0;
                        }
                        else if(unvisited_exits[0] == 1){
                            angle_to_rotate = M_PI_2;
                            ROS_INFO("Chose left");
                        }
                        else if(unvisited_exits[2] == 1){
                            angle_to_rotate = -M_PI_2;
                            ROS_INFO("Chose right");
                        }
                    }

                } else {
                    // dead end, go back the way you came
                    angle_to_rotate = M_PI;
                }

                rotateAngle(angle_to_rotate);
                // drive forward until in corridor
                driveStraight(0.8);
                updateWalls();
                if(wall_estimates_[0].r < 1e-3 && wall_estimates_[1].r < 1e-3 && center_range_ > 5.0)
                    current_state_ = States::ESCAPED;
                else
                    current_state_ = States::CORRIDOR;
                break;
            }

            case States::ESCAPED:{
                ROS_INFO("Robot escaped maze!");
                stop();
                last_state_ = current_state_;
                break;
            }
        }
    }

    void TurtleBotMaze::followWall() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.2;
        //while(current_scan_.header.seq == 0)
            //ros::spinOnce();
        //update_walls();
        //last_wall_update_ = ros::Time::now();
        while (ros::ok()) {
            //move_cmd.angular.z = k_ * heading_error_; //simple P-controller
            //heading_error_ = wall_estimates_[1].a - (current_pose_.h + M_PI_2);
            move_cmd.angular.z = pid_->runControlHE(0.6 - left_range_, ros::Time::now().toSec(), heading_error_);
            //ROS_INFO("LR: %f HE: %f", left_range_, heading_error_);
            vel_publisher_.publish(move_cmd);
            ros::spinOnce();
            if(ros::Time::now().toSec() - last_wall_update_.toSec() > 1.0){
                if (left_range_ < 1.2 && right_range_ < 1.2){
                    updateWalls();
                    if (stableEndpointEstimate()){
                        ROS_INFO("STABLE ENDPOINT ESTIMATE");
                        double min_distance = std::min(distance(current_pose_.p, wall_estimates_[0].p_e), distance(current_pose_.p, wall_estimates_[1].p_e));
                        if(min_distance - last_min_distance_ > 1e-3){
                            ROS_INFO("CLEARED WALL %f %f", min_distance, last_min_distance_);
                        }
                        last_min_distance_ = min_distance;
                    }

                }else{
                    last_min_distance_ = 1e3;
                    wd_->resetMedians();
                    resetWallEstimates();
                }
                last_wall_update_ = ros::Time::now();
            }
            loop_rate_->sleep();
            if (center_range_ < 0.5) { //too close to wall in front
                stop();
                if (left_range_ < right_range_)
                    rotateAngle(-M_PI_2);
                else
                    rotateAngle(M_PI_2);
                pid_->reset(ros::Time::now().toSec());
            }
        }
    }

    void TurtleBotMaze::stop() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
        vel_publisher_.publish(move_cmd);
    }

    void TurtleBotMaze::driveStraight(double distance_in) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = distance_in >= 0 ? 0.2 : -0.2;
        move_cmd.angular.z = 0.0;
        double distance_current = 0;
        while (std::fabs(distance_current) < std::fabs(distance_in) && center_range_ > 0.5) {
            ros::spinOnce();
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            distance_current += move_cmd.linear.x * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }
        // TODO: stop if center sensor is blocked
    }

    void TurtleBotMaze::rotateAngle(double angle_in) {
        // align first with nearest compass point
        double align_angle = angleToNearestCompassPoint();
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = align_angle >= 0 ? 0.5 : - 0.5;
        double angle_current = 0;
        while (std::fabs(angle_current) < std::fabs(align_angle)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }
        move_cmd.angular.z = angle_in >= 0 ? 0.5 : -0.5;
        angle_current = 0;
        while (std::fabs(angle_current) < std::fabs(angle_in)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
            //ROS_INFO("Rotating: %f", angle_current);
        }
    }

    double TurtleBotMaze::angleToNearestCompassPoint(){
        if(std::fabs(current_pose_.h) < M_PI_4){ // robot traveling in positive x direction
            return -current_pose_.h;
        }
        if(std::fabs(std::fabs(current_pose_.h) - M_PI) < M_PI_4){ // robot traveling in negative x direction
            if(current_pose_.h > 0)
                return M_PI - current_pose_.h;
            return -M_PI - current_pose_.h;
        }
        if(std::fabs(current_pose_.h - M_PI_2) < M_PI_4){ // robot traveling in positive y direction
            return M_PI_2 - current_pose_.h;
        }
        if(std::fabs(current_pose_.h + M_PI_2) < M_PI_4){ // robot traveling in negative y direction
            return -M_PI_2 - current_pose_.h;
        }
    }

    void TurtleBotMaze::callbackLaser(const sensor_msgs::LaserScan &msg) {

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

    void TurtleBotMaze::callbackPose(const nav_msgs::Odometry &msg) {

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

    void TurtleBotMaze::updateWalls() {
        if (current_scan_.header.seq == 0)
            return;

        std::vector<double> angles(hokuyo_num_ranges_, 0.0);
        std::vector<double> ranges(hokuyo_num_ranges_, 0.0);

        for(int i = 0; i < hokuyo_num_ranges_; ++i){
            angles[i] = i * current_scan_.angle_increment;
            ranges[i] = current_scan_.ranges[i];
        }

        wd_->updateWalls(current_pose_, ranges, angles);
        ph_->addPoint(current_pose_.p);
        //ph_->PrintPoints();



        ROS_INFO("Pose: x %f y %f h %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h);
        wall_detect_record_ << current_pose_.p.x << "," << current_pose_.p.y << "," << current_pose_.h << ",";
        /*int cc = 0;
        for (const auto &w : wd_->GetWalls()) {
            ROS_INFO("Walls: r %f a %f x0 %f y0 %f x1 %f y1 %f n %d", w.r, w.a, w.p_c.x, w.p_c.y, w.p_e.x, w.p_e.y, ++cc);
        }*/
        wd_->getWalls(wall_estimates_[1], wall_estimates_[0]);
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

    std::vector<int> TurtleBotMaze::checkUnvisitedExits(const std::vector<int> &open_exits){

        // TODO: how to check for escaped state?

        std::vector<int> unvisited_exits(3, 0);

        // check if open exits already visited
        //ROS_INFO("Pose: x %f y %f h %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h);
        if (open_exits[0]) {
            Point ll = transformToGlobal({-1.0, 2.5}, current_pose_);
            Point ur = transformToGlobal({1.0, 0.5}, current_pose_);
            if (!ph_->anyInRectangle(ll, ur)) {
                unvisited_exits[0] = 1;
                //ROS_INFO("Left exit already visited");
            }
        }
        if (open_exits[1]) {
            Point ll = transformToGlobal({0.5, 1.0}, current_pose_);
            Point ur = transformToGlobal({2.5, -1.0}, current_pose_);
            if (!ph_->anyInRectangle(ll, ur)) {
                unvisited_exits[1] = 1;
                //ROS_INFO("Center exit already visited");
            }
        }
        if (open_exits[2]) {
            Point ll = transformToGlobal({-1.0, -0.5}, current_pose_);
            Point ur = transformToGlobal({1.0, -2.5}, current_pose_);
            if (!ph_->anyInRectangle(ll, ur)) {
                unvisited_exits[2] = 1;
                //ROS_INFO("Right exit already visited");
            }
        }
        return unvisited_exits;
    }

    bool TurtleBotMaze::stableEndpointEstimate() { // checks both left wall and right wall endpoint estimates are consistent
        // TODO: maybe need just either endpoint estimate to be reliable? (ideally the nearest one to robot)
        static int stable_count = 0;
        static Point last_right_point, last_left_point;
        if(std::fabs(wall_estimates_[0].p_e.x) > 1e-3 && std::fabs(wall_estimates_[1].p_e.x) > 1e-3){
            if(distance(last_right_point, wall_estimates_[0].p_e) < 0.1 && distance(last_left_point, wall_estimates_[1].p_e) < 0.1)
                ++stable_count;
        }else{
            stable_count = 0;
        }
        last_right_point = wall_estimates_[0].p_e;
        last_left_point = wall_estimates_[1].p_e;
        return stable_count >= 2;
    }

    double TurtleBotMaze::stableDesiredHeading() { // TODO : make bool?
        static int left_stable_count = 0, right_stable_count = 0;
        static double last_left_wall_est = M_PI_4, last_right_wall_est = M_PI_4;

        if(wall_estimates_[0].r > 0.1){ // right wall valid
            if(right_stable_count < 3){
                if(angleDifference(last_right_wall_est, wall_estimates_[0].a) < 0.1)
                    ++right_stable_count;
                last_right_wall_est = wall_estimates_[0].a;
            }
            if(right_stable_count == 3){
                if(angleDifference(last_right_wall_est, wall_estimates_[0].a) > 0.1){
                    //right_stable_count = 0;
                }else{
                    //ROS_INFO("HEADING RIGHT WALL %f", WrapAngle(last_right_wall_est + M_PI_2));
                    return last_right_wall_est + M_PI_2;
                }
            }
        }else{
            last_right_wall_est = M_PI_4;
            right_stable_count = 0;
        }

        if(wall_estimates_[1].r > 0.1){ // left wall valid
            if(left_stable_count < 3){
                if(angleDifference(last_left_wall_est, wall_estimates_[1].a) < 0.1)
                    ++left_stable_count;
                last_left_wall_est = wall_estimates_[1].a;
            }
            if(left_stable_count == 3){
                if(angleDifference(last_left_wall_est, wall_estimates_[1].a) > 0.1){
                    //left_stable_count = 0;
                }else{
                    //ROS_INFO("HEADING LEFT WALL %f", WrapAngle(last_left_wall_est - M_PI_2));
                    return last_left_wall_est - M_PI_2;
                }
            }
        }else{
            last_left_wall_est = M_PI_4;
            left_stable_count = 0;
        }

        return current_pose_.h;
    }

    double TurtleBotMaze::stableDesiredPosition(bool use_x, int error_sign){ // TODO : make bool, make it jive better with cases code above?
        static int left_stable_count = 0, right_stable_count = 0;
        static double last_right_point = 0, last_left_point = 0;

        // assume heading doesn't change much throughout this process
        double current_right_point = wall_estimates_[0].p_c.x;
        double current_left_point = wall_estimates_[1].p_c.x;
        double current_pose_point = current_pose_.p.x;
        if(use_x) {// robot facing along global x-axis, care about y-coordinates
            current_right_point = wall_estimates_[0].p_c.y;
            current_left_point = wall_estimates_[1].p_c.y;
            current_pose_point = current_pose_.p.y;
        }

        if(std::fabs(current_right_point) > 1e-3){ // right wall valid
            if(right_stable_count < 3){
                if(std::fabs(last_right_point - current_right_point) < 0.1)
                    ++right_stable_count;
                last_right_point = current_right_point;
            }
            if(right_stable_count == 3 && std::fabs(last_right_point - current_right_point) > 0.1){

            }
                //right_stable_count = 0;
        }else{
            last_right_point = 0;
            right_stable_count = 0;
        }

        if(std::fabs(current_left_point) > 1e-3){ // left wall valid
            if(left_stable_count < 3){
                if(std::fabs(last_left_point - current_left_point) < 0.1)
                    ++left_stable_count;
                last_left_point = current_left_point;
            }
            if(left_stable_count == 3 && std::fabs(last_left_point - current_left_point) > 0.1){

            }
                //left_stable_count = 0;
        }else{
            last_left_point = 0;
            left_stable_count = 0;
        }

        int wall_choice = 0; // -1 for left, 1 for right, 0 for neither
        if(left_stable_count == 3 && right_stable_count == 3){
            if(std::fabs(last_left_point - last_right_point) < 2.0) { // you're in a corridor
                ROS_INFO("CORRIDOR CENTERING %f", (last_left_point + last_right_point)/2 - current_pose_point);
                return (last_left_point + last_right_point) / 2;
            }
            // otherwise, go close to whichever wall robot is already nearest to
            if(std::fabs(current_pose_point - last_left_point) < std::fabs(current_pose_point - last_right_point))
                wall_choice = -1;
            else
                wall_choice = 1;
        }

        if(wall_choice == 1 || (right_stable_count == 3 && left_stable_count < 3)){
            if(std::fabs(last_right_point - current_pose_point) < 1.2){
                ROS_INFO("HUG RIGHT WALL %f %f %f %d", last_right_point, current_pose_point, last_right_point + error_sign*0.75 - current_pose_point, error_sign);
                return last_right_point + error_sign * 0.75;
            }
        }
        if(wall_choice == -1 || (left_stable_count == 3 && right_stable_count < 3)) {
            if (std::fabs(last_left_point - current_pose_point) < 1.2) {
                ROS_INFO("HUG LEFT WALL %f %f %f %d", last_left_point, current_pose_point,
                         last_left_point - error_sign * 0.75 - current_pose_point, error_sign);
                return last_left_point - error_sign * 0.75;
            }
        }
        return 0.0;

    }

    void TurtleBotMaze::resetWallEstimates() {
        wall_estimates_[0].r = 0;
        wall_estimates_[0].a = 0;
        wall_estimates_[0].p_c = Point();
        wall_estimates_[0].p_e = Point();
        wall_estimates_[1].r = 0;
        wall_estimates_[1].a = 0;
        wall_estimates_[1].p_c = Point();
        wall_estimates_[1].p_e = Point();
    }

} // namespace turtlebot_maze

