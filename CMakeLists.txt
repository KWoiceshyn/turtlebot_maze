cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)
project(turtlebot_maze)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES position_history wall_detection turtlebot_maze
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(position_history src/position_history.cpp)
target_link_libraries(position_history ${catkin_LIBRARIES})

add_library(wall_detection src/wall_detection.cpp)
target_link_libraries(wall_detection ${catkin_LIBRARIES})

add_library(turtlebot_maze src/turtlebot_maze.cpp)
target_link_libraries(turtlebot_maze wall_detection position_history ${catkin_LIBRARIES})

add_executable(turtlebot_maze_node src/turtlebot_maze_node.cpp)
target_link_libraries(turtlebot_maze_node turtlebot_maze ${catkin_LIBRARIES})
