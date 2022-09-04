# SofAR assignment
The assignment consists in detecting tables as active obstacles for navigation using both the RGB-D sensor and the Lidar scan with the final aim of correctly mapping raised obstacles in the 2D occupancy grid.
The robot used in simulation here is Tiago.

## Installing and running
To run our code, first you need to install the following packages (make sure to check melodic version):  
[Tiago ROS tutorial](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS),  
[Gazebo models](https://github.com/osrf/gazebo_models),  
[Laserscan merger](https://github.com/robotics-upo/laserscan_merger)

At first, install the current repository in your ROS workspace by executing in terminal:
```
cd tiago_public_ws/src
git clone https://github.com/antob98/Tiago_Head_Control.git
```
At last, download this package and build the workspace by executing `catkin build` command.

Now, to run this version of the simulation digit respectively in three different terminals:
```
roslaunch final_assignment simulation_gmapping.launch
roslaunch final_assignment move_base.launch
roslaunch final_assignment interface.launch
```
Gazebo and RViz environment is now available.

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
