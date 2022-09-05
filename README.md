# SofAR assignment
The assignment consists in detecting tables as active obstacles for navigation using both the RGB-D sensor and the Lidar scan with the final aim of correctly mapping raised obstacles in the 2D occupancy grid.
The robot used in simulation here is Tiago.

## Installing
To run our code, first you need to install the following packages (make sure to check melodic version):  
[Tiago ROS tutorial](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS),  
[Gazebo models](https://github.com/osrf/gazebo_models),  
[Laserscan merger](https://github.com/robotics-upo/laserscan_merger)

At first, install the current repository on your Desktop, for example, by executing in terminal:
```
git clone https://github.com/antob98/Tiago_Head_Control.git
```

After downloading this repo, move the package `my_head_control` in the `src` folder of `tiago_public_ws`.  
Then, move the `.world` files in this `worlds` folder inside `tiago_public_ws/src/tiago_simulation/tiago_gazebo/worlds` while in `tiago_public_ws/src/tiago_simulation/tiago_gazebo/models` move all the elements cloned by [Gazebo models](https://github.com/osrf/gazebo_models).  
At last, in `tiago_simulation/tiago_2dnav_gazebo` create a folder `src` and move here the file `interface.cpp` while in the `launch` folder move the file `tiago_mapping_2.launch` of this repo.

Finally, build the workspace by executing the commands: `cd tiago_public_ws`, then `catkin build`, then `source .devel/setup.bash` in the terminal.  
A detailed UML of the packages and files of interest is here presented:

![Uml_Diagram](https://user-images.githubusercontent.com/93495918/188476017-ffcb6288-fb9c-427a-a2b2-27c30ad71ca1.png)

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
Wait until the message "done operation: 0.0" is printed on the terminal, or at least until the robot's head inclination is set.
Finally, simply input your goals. You should be able to explore the whole map without ever colliding with tables, even if your goal is set under them. If it cannot reach the goal, it simply stops.

By running this solution with our custum worlds, we also observed that Tiago has difficulties with wide obstacles because the RGB-D camera data aren't saved in the occupancy grid of gmapping. So, for this type of obstacles, Tiago only sees part of it and may remain stuck in trying to go around it.
To solve this problem, we implemented the following solution: table mapping.

#### Table mapping
In this implementation, we merged data from Lidar sensor and RGB-D camera with [Laserscan merger](https://github.com/robotics-upo/laserscan_merger) in order to have only one laser topic used from gmapping to build the occupancy grid and the map.
Note that the RGB-D camera data have to be converted in laserscan and this is done by the package `pointcloud_to_laserscan`.

Here, there is also the possibility to save the map thanks to a custom interface in order to use the same map with other robots, such that the obstacle position is known a priori.

To use this code, you need to take from the rgbd_mapping directory the following files:

* tiago_mapping_2.launch. Put it in this path 
```
tiago_public_ws/src/tiago_simulation/tiago_2dnav_gazebo/launch
```

* package.xml and CMakeLists.txt. Put them in this path (substitute the previous files already present)
```
tiago_public_ws/src/tiago_simulation/tiago_2dnav_gazebo
```

*  interface.cpp . Create a src directory in the path above and put inside it the interface.cpp file

* laserscan_merger.launch. Put it in this path (substitute the previous file already present)
```
tiago_public_ws/src/laserscan_merger/launch
```

* mapping.launch Put it in this path (substitute the previous file already present)
```
tiago_public_ws/src/tiago_navigation/tiago_2dnav/launch
```

Lastly open a terminal and input:
`cd tiago_public_ws/`

Then, input:

```
catkin build
```
```
source ./devel/setup.bash
```
```
roslaunch tiago_2dnav_gazebo tiago_mapping_2.launch public_sim:=true
```

## Conclusions and future implementation
With our work we managed to solve the issue regarding obstacle avoidance with tables within the default tutorial map.
Regarding the custom maps we created, we observed that also other type of obstacles can be avoided correctly by Tiago.

Moreover, we also managed to correctly save the maps with our interface, also with the presence of the tables in the occupancy grid.

As future implementation, the mapping process could be stabilized (when the rgbd data are merged with the lidar sensor data) in order to obtain a more accurate map, usable also for other type of robots. 
