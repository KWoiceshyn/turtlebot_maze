#include "turtlebot_maze/turtlebot_maze.h"

namespace turtlebot_maze {

    TurtleBotMaze::TurtleBotMaze(ros::NodeHandle &nh)
            : nh_{nh},
              loop_rate_{new ros::Rate(10)},
              center_range_{1.0},
              current_state_{States::INITIALIZE},
              last_state_{States::INITIALIZE}
    {
        init();
        wd_ = std::make_unique<WallDetection>(4.0);
        ph_ = std::make_unique<PositionHistory>();
        pid_ = std::make_unique<PID>(0.5, 0.05, 1.0);
        wall_estimates_ = {WallModel(), WallModel()};
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
    }

    void TurtleBotMaze::stateMachine() {

        switch(current_state_){

            case States::INITIALIZE: {
                getUpdatedWalls();
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
                static double last_left_range = 1e3, last_right_range = 1e3, last_min_distance = 1e3;
                if(last_state_ != current_state_){
                    ROS_INFO("Entered Corridor state");
                    last_state_ = current_state_;
                    getUpdatedWalls();
                    last_left_range = left_range_;
                    last_right_range = right_range_;
                    last_min_distance = 1e3;
                }
                geometry_msgs::Twist move_cmd;
                move_cmd.linear.x = 0.2;
                double heading_error = angleDifference(stableDesiredHeading(), current_pose_.h);
                if(std::fabs(heading_error) > M_PI_4){
                    // don't use heading error for control if it's very large; robot is not nicely oriented to wall
                    ROS_INFO("HEADING ERROR %f", heading_error);
                    heading_error = 0.0;
                }

                // TODO: make error wrt any line rather than 4 cardinal directions?
                double position_error = stableDesiredPosition(current_pose_);
                move_cmd.angular.z = pid_->runControlHE(position_error, ros::Time::now().toSec(), heading_error);

                vel_publisher_.publish(move_cmd);
                if(ros::Time::now().toSec() - last_wall_update_.toSec() > 0.2){
                    getUpdatedWalls();
                    if (stableEndpointEstimate()){
                        ROS_INFO("STABLE WALL ENDPOINT ESTIMATE");
                        double min_distance = std::min(distance(current_pose_.p, wall_estimates_[0].p_e), distance(current_pose_.p, wall_estimates_[1].p_e));
                        // check if distance from robot to nearest wall endpoint starts to increase, then you have cleared the end of the wall
                        if(min_distance - last_min_distance > 1e-3){
                            ROS_INFO("CLEARED WALL %f %f", min_distance, last_min_distance);
                            stop();
                            current_state_ = States::INTERSECTION;
                        }
                        last_min_distance = min_distance;
                    }
                }

                // additional check for intersection state if either side laser detects a sudden increase in range
                if(center_range_ < 0.8 || left_range_ - last_left_range > 1.0 || right_range_ - last_right_range > 1.0){
                    stop();
                    current_state_ = States::INTERSECTION;
                }
                last_left_range = left_range_;
                last_right_range = right_range_;
                break;
            }

            case States::INTERSECTION: {
                if (last_state_ != current_state_) {
                    ROS_INFO("Entered Intersection state");
                    last_state_ = current_state_;
                    pid_->reset(ros::Time::now().toSec());
                    wd_->resetMedians();
                    resetWallEstimates();
                    stableDesiredHeading(); // reset the static variables
                    stableDesiredPosition(current_pose_);
                }
                // drive forward a bit to clear walls
                //rotateToNearestCompassPoint(current_pose_.h);
                if(center_range_ > 1.0)
                    driveStraight(0.5);
                ros::spinOnce();

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

                // align first with nearest compass point, to correct accumulating heading drift
                rotateToNearestCompassPoint(current_pose_.h);
                rotateAngle(angle_to_rotate);

                driveStraight(0.8); // drive forward until in corridor
                getUpdatedWalls();

                // TODO CHECK ALL LASERS
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
    /*
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
    }*/

    void TurtleBotMaze::stop() {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = 0.0;
        vel_publisher_.publish(move_cmd);
    }

    void TurtleBotMaze::driveStraight(double distance) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = distance >= 0.0 ? 0.2 : -0.2;
        move_cmd.angular.z = 0.0;
        double distance_current = 0.0;
        // check for collision
        while (std::fabs(distance_current) < std::fabs(distance) && center_range_ > 0.5) {
            ros::spinOnce();
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            distance_current += move_cmd.linear.x * 0.1;
        }
    }

    void TurtleBotMaze::rotateAngle(double angle) {
        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = angle >= 0.0 ? 0.5 : -0.5;
        double angle_current = 0.0;
        while (std::fabs(angle_current) < std::fabs(angle)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
        }
    }

    TurtleBotMaze::Directions TurtleBotMaze::nearestCompassPoint(double heading){
        if(std::fabs(heading) < M_PI_4){ // robot traveling in positive x direction
            return Directions::POS_X;
        }
        if(std::fabs(std::fabs(heading) - M_PI) < M_PI_4){ // robot traveling in negative x direction
            return Directions::NEG_X;
        }
        if(std::fabs(heading - M_PI_2) < M_PI_4){ // robot traveling in positive y direction
            return Directions::POS_Y;
        }
        if(std::fabs(heading + M_PI_2) < M_PI_4){ // robot traveling in negative y direction
            return Directions::NEG_Y;
        }
    }

    void TurtleBotMaze::rotateToNearestCompassPoint(double heading){
        auto dir = nearestCompassPoint(heading);
        double align_angle = 0.0;
        switch(dir){
            case Directions::POS_X:{
                align_angle = -heading;
                break;
            }
            case Directions::NEG_X:{
                if(heading > 0.0)
                    align_angle =  M_PI - heading;
                else
                    align_angle = -M_PI - heading;
                break;
            }
            case Directions::POS_Y:{
                align_angle = M_PI_2 - heading;
                break;
            }
            case Directions::NEG_Y:{
                align_angle = -M_PI_2 - heading;
                break;
            }
        }

        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.0;
        move_cmd.angular.z = align_angle >= 0.0 ? 0.5 : - 0.5;
        double angle_current = 0.0;
        while (std::fabs(angle_current) < std::fabs(align_angle)) {
            vel_publisher_.publish(move_cmd);
            loop_rate_->sleep();
            angle_current += move_cmd.angular.z * 0.1;
        }
    }

    void TurtleBotMaze::callbackLaser(const sensor_msgs::LaserScan &msg) {
        left_range_ = msg.ranges[left_laser_idx_];
        center_range_ = msg.ranges[center_laser_idx_];
        right_range_ = msg.ranges[right_laser_idx_];
        current_scan_ = msg;
    }

    void TurtleBotMaze::callbackPose(const nav_msgs::Odometry &msg) {
        tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.p.x = msg.pose.pose.position.x;
        current_pose_.p.y = msg.pose.pose.position.y;
        current_pose_.h = yaw;
    }

    void TurtleBotMaze::getUpdatedWalls() {
        if (current_scan_.header.seq == 0)
            return;

        std::vector<double> angles(hokuyo_num_ranges_, 0.0);
        std::vector<double> ranges(hokuyo_num_ranges_, 0.0);

        for(int i = 0; i < hokuyo_num_ranges_; ++i){
            angles[i] = i * current_scan_.angle_increment;
            ranges[i] = current_scan_.ranges[i];
        }

        wd_->updateWalls(current_pose_, ranges, angles); // send latest scan data to wall_detector
        ph_->addPoint(current_pose_.p); // add current position to history
        wd_->getWalls(wall_estimates_[1], wall_estimates_[0]); // get best estimate wall models from wall_detector using a median calculation

        ROS_INFO("Pose: x %f y %f h %f", current_pose_.p.x, current_pose_.p.y, current_pose_.h);
        ROS_INFO("RWall: r %f a %f xc %f yc %f xe %f ye %f", wall_estimates_[0].r, wall_estimates_[0].a, wall_estimates_[0].p_c.x, wall_estimates_[0].p_c.y,
                 wall_estimates_[0].p_e.x, wall_estimates_[0].p_e.y);
        ROS_INFO("LWall: r %f a %f xc %f yc %f xe %f ye %f", wall_estimates_[1].r, wall_estimates_[1].a, wall_estimates_[1].p_c.x, wall_estimates_[1].p_c.y,
                 wall_estimates_[1].p_e.x, wall_estimates_[1].p_e.y);
        std::cout << "-----------------------------------\n";

        last_wall_update_ = ros::Time::now();
    }

    std::vector<int> TurtleBotMaze::checkUnvisitedExits(const std::vector<int> &open_exits){

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

    bool TurtleBotMaze::stableEndpointEstimate() {

        // TODO: maybe need just either endpoint estimate to be reliable? (ideally the nearest one to robot)
        static int stable_count = 0;
        static Point last_right_point, last_left_point;
        if(std::fabs(wall_estimates_[0].p_e.x) > 1e-3 && std::fabs(wall_estimates_[1].p_e.x) > 1e-3){ // TODO: use nan instead of 0?
            if(distance(last_right_point, wall_estimates_[0].p_e) < 0.1 && distance(last_left_point, wall_estimates_[1].p_e) < 0.1)
                ++stable_count;
        }else{
            stable_count = 0;
        }
        last_right_point = wall_estimates_[0].p_e;
        last_left_point = wall_estimates_[1].p_e;
        // make sure we have at least a few wall endpoint estimates in a row that are very near each other
        return stable_count >= 2;
    }

    double TurtleBotMaze::stableDesiredHeading() {

        static int stable_count[2] = {0, 0};
        static double last_wall_est[2] = {M_PI_4, M_PI_4}; // TODO: better choice of invalid value?

        for(auto i = 0; i < 2; ++i){ // 0 = right, 1 = left
            int mult = i == 0 ? 1 : -1;
            if(wall_estimates_[i].r > 0.1){ // wall valid
                if(stable_count[i] < 3){
                    if(angleDifference(last_wall_est[i], wall_estimates_[i].a) < 0.1)
                        ++stable_count[i];
                    last_wall_est[i] = wall_estimates_[i].a;
                }
                if(stable_count[i] == 3){
                    if(angleDifference(last_wall_est[i], wall_estimates_[i].a) < 0.1){
                        return last_wall_est[i] + mult * M_PI_2;
                    }
                }
            }else{
                last_wall_est[i] = M_PI_4;
                stable_count[i] = 0;
            }
        }
        return current_pose_.h;
    }

    double TurtleBotMaze::stableDesiredPosition(const Pose& pose){

        static int stable_count[2] = {0, 0};
        static double last_point[2] = {0.0, 0.0};

        auto dir = nearestCompassPoint(pose.h);
        double actual_position = 0.0;
        int error_sign = 0;

        switch(dir){
            case Directions::POS_X:{
                error_sign = 1;
                actual_position = pose.p.y;
                break;
            }
            case Directions::NEG_X:{
                error_sign = -1;
                actual_position = pose.p.y;
                break;
            }
            case Directions::POS_Y:{
                error_sign = -1;
                actual_position = pose.p.x;
                break;
            }
            case Directions::NEG_Y:{
                error_sign = 1;
                actual_position = pose.p.x;
            }
        }

        // assume heading doesn't change much throughout this process
        double current_point[2] = {wall_estimates_[0].p_c.x, wall_estimates_[1].p_c.x};
        double current_pose_point = current_pose_.p.x;
        if(dir == Directions::POS_X || dir == Directions::NEG_X) {// robot facing along global x-axis, use y-coordinates
            current_point[0] = wall_estimates_[0].p_c.y;
            current_point[1] = wall_estimates_[1].p_c.y;
            current_pose_point = current_pose_.p.y;
        }

        for(auto i = 0; i < 2; ++i){ // 0 = right, 1 = left
            if(std::fabs(current_point[i]) > 1e-3){ // wall valid
                if(stable_count[i] < 3){
                    if(std::fabs(last_point[i] - current_point[i]) < 0.1)
                        ++stable_count[i];
                    last_point[i] = current_point[i];
                }
            }else{
                last_point[i] = 0.0;
                stable_count[i] = 0;
            }
        }

        int wall_choice = 0; // -1 for left, 1 for right, 0 for neither
        if(stable_count[0] == 3 && stable_count[1] == 3){
            if(std::fabs(last_point[0] - last_point[1]) < 2.0) { // you're in a corridor
                ROS_INFO("CORRIDOR CENTERING %f", (last_point[0] + last_point[1])/2 - current_pose_point);
                double desired_position = (last_point[0] + last_point[1]) / 2;
                return error_sign * (desired_position - actual_position);
            }
            // otherwise, drive close to whichever wall robot is already nearest to
            if(std::fabs(current_pose_point - last_point[1]) < std::fabs(current_pose_point - last_point[0]))
                wall_choice = -1;
            else
                wall_choice = 1;
        }

        if(wall_choice == 1 || (stable_count[0] == 3 && stable_count[1] < 3)){
            if(std::fabs(last_point[0] - current_pose_point) < 1.2){
                ROS_INFO("HUG RIGHT WALL w: %f r: %f err: %f", last_point[0], current_pose_point, last_point[0] + error_sign*0.75 - current_pose_point);
                double desired_position =  last_point[0] + error_sign * 0.75;
                return error_sign * (desired_position - actual_position);
            }
        }
        if(wall_choice == -1 || (stable_count[1] == 3 && stable_count[0] < 3)) {
            if (std::fabs(last_point[1] - current_pose_point) < 1.2) {
                ROS_INFO("HUG LEFT WALL w: %f r: %f err: %f", last_point[1], current_pose_point, last_point[1] - error_sign * 0.75 - current_pose_point);
                double desired_position = last_point[1] - error_sign * 0.75;
                return error_sign * (desired_position - actual_position);
            }
        }
        return 0.0; // return 0 if wall models don't admit good control
    }

    void TurtleBotMaze::resetWallEstimates() {
        wall_estimates_[0] = WallModel();
        wall_estimates_[1] = WallModel();
    }

} // namespace turtlebot_maze

