# Turtlebot Maze
This is a ROS package that allows a simulated [Turtlebot](http://wiki.ros.org/turtlebot_gazebo) with a laser rangefinder to navigate its way out of a maze. I chose this project to explore a few topics in planning, navigation, and lidar-based perception. Its key algorithmic developments are wall model estimation based on the Hough Transform, and an exploration state machine with position history.

## Description
### State Machine
The algorithm is based on the assumption that a maze consists of straight walls oriented at 90 degrees to one another, and that these form corridors or hallways of approximately equal width. Thus, the robot can be in one of two states: corridor, or intersection.
#### 1) Corridor State
In a corridor, the robot uses wall model estimation to assess its position and bearing relative to the walls on either side, as well as how far it is from the end points of the walls (e.g. corners) -- i.e. how far it is from the end of the corridor. It uses this information in a PID controller to minimize its lateral position error and heading error.
#### 2) Intersection State
An intersection is where two to four corridors join, or a corridor reaches a dead end. The intersection state is entered when the robot passes a wall endpoint estimated in the corridor state. First, the laser is used to determine which of the three possible exit directions (not including the one arrived from) are open, and which are blocked by walls. Of the open exits, the position history is checked to see if any has been already visited. The order of preference differs between visited and unvisited directions, to avoid getting stuck in an endless loop.
### Wall Model Estimation
In the corridor state, it is assumed that the robot has a wall on its left side and a wall on its right side. Some accommodations are made for very short wall segments. A wall model consists of the following information:
- range: distance from the robot's center (laser) to the nearest point on the wall
- angle: orientation of the wall in the global(map) frame
- near endpoint: (x,y) position of nearest point on wall that laser can detect (this lies on the robot's body y-axis since the laser's field of view is 180 degrees)
- far endpoint: (x,y) position of the furthest point on the wall that the laser can detect (e.g. a corner or the end of the wall)

The ranges and angles of the walls are estimated using a Hough transform. The endpoints are found by traversing the laser ranges in the scan until a point is found that does not match the model. Reliability of the endpoint estimates is improved by using median values of several previous estimates.
### Position History
As the robot explores the maze, it records its (x,y) position at a regular time interval. Since its driving speed in a corridor is constant, these points are spaced approximately uniformly. In an intersection, the possible exit corridors are assessed. A rectangle is projected a short distance into each of these. If there are no recorded points in the rectangle, the corridor is unvisited.
## Usage
I developed this using ROS [Kinetic](http://wiki.ros.org/kinetic) in Ubuntu 16.04. Prerequisites installations are the [turtlebot_simulator](http://wiki.ros.org/turtlebot_simulator) package as well as [Gazebo](http://gazebosim.org/) version 7.
```
sudo apt-get install ros-kinetic-turtlebot-simulator
sudo apt-get install gazebo7
```
### To add the Hokuyo laser
I swapped the Kinect sensor on the default Turtlebot Gazebo model for a Hokuyo laser rangefinder. A tutorial can be found [here](https://joshi-bharat.github.io/posts/adding-hokuyo-lasers). However, I substituted the following description in the file `turtlebot_gazebo.urdf.xacro`: 
```
<horizontal>
    <samples>360</samples>
    <resolution>1</resolution>
    <min_angle>-1.570796</min_angle>
    <max_angle>1.570796</max_angle>
</horizontal>
```

### To run the simulation
To launch Gazebo with the Turtlebot sim:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
To spawn the maze model:
```
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/turtlebot_maze/models/maze_tb/model.sdf -sdf -model maze -x 1 -y 0
```
To run the maze solver:
```
roslaunch turtlebot_maze tb_maze.launch
```