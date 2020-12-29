#include "../include/turtlebot_maze/turtlebot_maze.h"
//#include "../include/turtlebot_maze/position_history.h"
//#include "../include/turtlebot_maze/geometric_utilities.h"
//#include "../include/turtlebot_maze/wall_detection.h"

#include <csignal>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>


void mySigintHandler(int sig)
{
    ROS_INFO("Stopping turtlebot");
    ros::shutdown();
}

using namespace turtlebot_maze;

int main(int argc, char **argv)
{
    ///////////////////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "turtlebot_maze", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    TurtleBotMaze tb_maze(nh);
    tb_maze.follow_wall();
    /*while(ros::ok()){
        ros::spinOnce();
        tb_maze.state_machine();
        tb_maze.loop_rate_->sleep();
    }*/



    ///////////////////////////////////////////////////////////////////////////////
    /*auto pHist = new PositionHistory();

    std::vector<Point> points {
            {-1.2, 0.0}, {0.0, -1.1}, {2.5, 0.0}, {2.5, 2.0}, {4.2, 1.0}
    };

    for(auto &p : points)
        pHist->AddPoint(p);

    auto r0 = pHist->AnyInRectangle(-1.0, 4.0, -1.0, 2.0);
    auto r1 = pHist->AnyInRectangle(0.0, 2.0, 0.0, 1.0);*/

    //////////////////////////////////////////////////////////////////////////////
    /*auto wDet = new WallDetection(4.0);
    Pose pose;
    pose.h = M_PI / 2.0;
    pose.p = {0.0, 0.0};

    std::vector<double> ranges;
    std::vector<double> angles;

    std::ifstream scan_file;
    scan_file.open("/home/kasper/catkin_ws/src/turtlebot_maze/scripts/sampleScan.txt");
    if(!scan_file.is_open()){
        printf("scan file not open \n");
        return 1;
    }
    std::string line;
    while(getline(scan_file, line)){
        std::istringstream iss(line);
        double angle, range;
        iss >> angle >> range;
        angles.push_back(angle);
        ranges.push_back(range);
        //std::cout << angles.back() << " " << ranges.back() << std::endl;
    }

    wDet->UpdateWalls(pose, ranges, angles);*/


    return 0;
}
