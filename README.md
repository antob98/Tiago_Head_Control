# SofAR assignment
The assignment consists in detecting tables as active obstacles for navigation using both the RGB-D sensor and the Lidar scan with the final aim of correctly mapping raised obstacles in the 2D occupancy grid.
The robot used in simulation here is Tiago.

## Installing
To run our code, first you need to install the following packages (make sure to check melodic version):  
[Tiago ROS tutorial](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS),  
[Gazebo models](https://github.com/osrf/gazebo_models),  
[Laserscan merger](https://github.com/robotics-upo/laserscan_merger)

At first, install the current repository in your ROS workspace by executing in terminal:
```
cd tiago_public_ws/src
git clone https://github.com/antob98/Tiago_Head_Control.git
```
(........)

At last, build the workspace by executing `catkin build` command and type in the terminal `source .devel/setup.bash`.


## Contents and running
### Simulation and behaviours
To run the default simulation of the tutorial open a terminal:
```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true
```
With this implementation, we have observed that Tiago collides with obstacles as tables whenever a goal is set under them.
To explore other Tiago behaviours, we run the simulation in other worlds created by us typing in the terminal:
```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true world:=tables
```
or
```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true worlds:=test1
```
We observed that Tiago has some difficulties with lower and rectangular tables because of the longer sides make it complex to fully detect the obstacle with the RGB-D camera.
To solve these problems, we implemented the following solution: head control.

#### Head control
In this implementation, we lowered the camera angulation and make Tiago able to turn its head in the same direction where it is turning.
In this way, we were able to correctly detect the tables.

To use this code, open a terminal and input:
`cd tiago_public_ws/`

Then, open a new window from the same terminal. In the first one, input:
```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true
```
In the second one, input:
```
roslaunch my_head_control test.launch
```
WAIT until the message "done operation: 0.0" is printed on the terminal, or at least until the robot's head inclination is set.
Finally, simply input your goals. You should be able to explore the whole map without ever colliding with tables, even if your goal is set under them. If it cannot reach the goal, it simply stops.

By running this solution with our custum worlds, we also observed that Tiago has difficulties with wide obstacles because the RGB-D camera data aren't saved in the occupancy grid of gmapping. So, for this type of obstacles, Tiago only sees part of it and may remain stuck in trying to go around it.
To solve this problem, we implemented the following solution: table mapping.

#### Table mapping
In this implementation, we merged data from Lidar sensor and RGB-D camera with [Laserscan merger](https://github.com/robotics-upo/laserscan_merger) in order to have only one laser topic used from gmapping to build the occupancy grid and the map.
Note that the RGB-D camera data have to be converted in laserscan and this is done by the package `pointcloud_to_laserscan`.

Here, there is also the possibility to save the map thanks to a custom interface in order to use the same map with other robots, such that the obstacle position is known a priori.

To use this code, open a terminal and input:
`cd tiago_public_ws/`

Then, open a new window from the same terminal. In the first one, input:
```
roslaunch tiago_2dnav_gazebo tiago_mapping_2.launch public_sim:=true
```

## Conclusions


## Simulation topics used
### Reach new goal ###
By publishing in `move_base/goal` topic, the user can give (x,y) position of the target he wants the robot to reach. For this purpouse the user need to access to the following fields of the topic:
* *goal*
* *target_pose* 
* *pose*
* *position*

And then the following fields can be modify:
* *x*,*y*: target new coordinates

This topic type is `move_base_msgs/MoveBaseActionGoal`.

### Goal status ###
By subscribing to `move_base/status` topic, the node has access to many fields among which `status_list` is used to get information about the state of the goal. In this boolean array two elements are important:
* *SUCCEDED*: is 1 whenever the goal is satisfied
* *REJECTED*: is 1 whenever the goal is unreachable

This topic type is `actionlib_msgs/GoalStatusArray`.

### Cancel goal ###
By publishing in `move_base/cancel` topic, the current goal can be cancelled both by the user and if the goal is unreachable. It only needs to publish an empty request.

This topic type is `actionlib_msgs/GoalID`.

### Scan obstacles ###
By subscribing to the `scan` topic, the node has access to many fields among which `rages` is used to get the information needed: it is an array of 721 elements which contains the distances from the nearest obstacles in a [0 180]° vision range.

This topic type is `sensor_msgs/LaserScan`.

### Velocity ###
By publishing into the `/cmd_vel` topic, the node can modify its fields:
* *linear*: linear velocity array
  * *x*,*y*: direction of the linear velocity
* *angular*: angular velocity array
  * *z*: direction of the angular velocity 

This topic type is `geometry_msgs/Twist`.

## Simulation custom service ##
A custum service `/service` handles communication between all nodes by passing information about:
* *state of the goal*: SUCCESSED or REJECTED
* *minimum distances from obstacles*: min_r, min_c and min_l
* *keyboard commands*: dir
* *control modality*: user driving or user assisted driving
