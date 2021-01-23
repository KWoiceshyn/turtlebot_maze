#include "turtlebot_maze/turtlebot_maze.h"

#include <csignal>
#include <iostream>


void mySigintHandler(int sig)
{
    ROS_INFO("Stopping turtlebot");
    ros::shutdown();
}

using namespace turtlebot_maze;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "turtlebot_maze_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    TurtleBotMaze tb_maze(nh);
    //tb_maze.followWall();
    while(ros::ok()){
        ros::spinOnce();
        tb_maze.stateMachine();
        if(tb_maze.getState() == turtlebot_maze::TurtleBotMaze::States::ESCAPED)
            break;
        tb_maze.sleep();
    }

    return 0;
}
