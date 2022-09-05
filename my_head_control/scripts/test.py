#! /usr/bin/env python

"""
Python code for the "my_head_control" node, an extension for ROS Tiago simulation.

This node aims at preventing the robot from colliding with tables due to the limitations of the RGB-D camera's cone of vision when a goal is set for it specifically under said tables. 

The robot should try to reach the goal, but without touching or dragging tables, then it should realize autonomously that it is unreachable, and stop.

We achieve this behavior by controlling the inclination of the robot's head, and by making it turn its head in the direction where it is trying to move, in order to always extend its cone of vision towards the lower part of where it will be facing.

INSTRUCTIONS:
To use this node, open a terminal and input:
cd tiago_public_ws/

Then, open a new window from the same terminal. In the first one, input:
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true

In the second one, input:
roslaunch my_head_control test.launch

WAIT until the message "done operation: 0.0" is printed on the terminal, or at least until the robot's head inclination is set.

Finally, simply input your goals. You should be able to explore the whole map without ever colliding with tables, even if your goal is set under them.
"""

import roslib
roslib.load_manifest('my_head_control')
import rospy
import actionlib
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadGoal
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo

"""
The send_hcgoal function sets and sends the correct message for the point_head_action server in order to change the facing direction of the RGB-D camera according to what we defined in X and Y. The fourth parameter is a damping factor for the timing instructions. We need it to be sure to exit the function only when the camera is fully turned in the right direction. It was also useful to experiment, so we kept it.
"""
def send_hcgoal(client,X,Y,r):
        windowName = String()
        windowName = "Inside of TIAGo's head"
        cameraFrame = String()
        cameraFrame = "/xtion_rgb_optical_frame"
        imageTopic = String()
        imageTopic = "/xtion/rgb/image_raw"
        cameraInfoTopic = String()
        cameraInfoTopic = "/xtion/rgb/camera_info"
	latestImageStamp = rospy.Time()
        pointStamped = PointStamped()
        goal = PointHeadGoal()

        pointStamped.header.frame_id = cameraFrame
        pointStamped.header.stamp = latestImageStamp
        pointStamped.point.x = X 
        pointStamped.point.y = Y 
        pointStamped.point.z = 1.0 
        goal.pointing_frame = cameraFrame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(0.5/r)
        goal.max_velocity = 1.5
        goal.target = pointStamped

        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(1.0/r))
        rospy.sleep(0.2/r)

        return X
"""
The turning function defines the logic with which tiago's head will be turned, based on the current state of the robot in motion and also with respect to the current facing direction of the RGB-D camera.
To start, it sets the p parameter of the my_head_state rosparameter to 0, to avoid possible errors, then it sets it to 1 (busy waiting, since ros parameters don't ensure synchronicity) at the end of the following procedure:
- if the robot is "turning left" and the facing direction of the camera is not left, it turns it to the left by X and changes the current d parameter of the my_head_state rosparameter to match the current facing direction of the camera; 
- if the robot is "turning right" the same thing happens with "right" instead of "left"; 
- if the robot is "facing front" and the facing direction of the camera is not front, then it turns to the left if it is currently on the right, viceversa otherwise;
- if none of these conditions apply (meaning that the robot is turning towards the same direction in which the camera is pointed right now, so there's no reason tu turn the camera) nothing is done.  

Note: the RGB-D camera has a resolution of 640x480 pixels, starting from pixel 0,0 in the upper left corner of the image and ending with pixel 640,480 in the lower right corner of the image. To make so that the head can be directed more easily, this number of pixels get converted into (roughly):
x - image width converted from (0)-(640) to (-0.61376)-(0.61376) 
y - image height converted from (0)-(480) to (-0.456729)-(0.456729)
This means that now the center of the image is in 0,0 thus: to turn left we give a negative command on x, to turn right we give a positive command instead; to face upwards we give a negative command on y, to face downwards we give a positive command instead.
"""
def turning(client,num,d):

        rospy.set_param("/my_head_state",{'p': 0, 'dir': d}) 
        print("current dir: ", d)
        #sx 0, front 1, dx 2
        
        Y = 0.1/6 #the camera slowly goes up, so at every step we pull it down a little
        if num == 1 and d > 0:
            X = -(0.61376/2)
            new_d = d-1
            print("next dir: ", new_d)
            wait_r = 2
            print("done operation: ",send_hcgoal(client,X,Y,wait_r)) 
        elif num == 2 and d < 2:
            X = 0.61376/2
            new_d = d+1
            print("next dir: ", new_d)
            wait_r = 2
            print("done operation: ",send_hcgoal(client,X,Y,wait_r))
        elif num == 3 and d != 1: 
            if d < 1:
                X = 0.61376/2
                new_d = d+1
            else:
                X = -(0.61376/2)
                new_d = d-1
            print("next dir: ", new_d)
            wait_r = 2
            print("done operation: ",send_hcgoal(client,X,Y,wait_r))
        else:
            new_d = d
            print("next dir: ", new_d)
            print("done operation: null")

        rospy.set_param("/my_head_state",{'p': 1, 'dir': new_d})
        while d != new_d:
            check = rospy.get_param("/my_head_state")
            d = check['dir']
"""
Callback function to the cmd_vel subscriber.
Each time a new message is published on cmd_vel, thus each time that a new motion command is given to tiago, it does the followings:
- It gets the ros parameter my_head_state which we defined in tiago_public_ws/src/my_head_control/param in the file hc_params.yaml; then it stores the two parameters of the list into the variables p (permission) and d (direction).
- It checks the p element of the parameter list, which can be set either at 0 or 1 during the execution of the code: 1 grants permission for acting on the head, 0 doesn't allow any action.
- If p is 1 then it checks if the z-component of the angular type from the msg received is set as higher, lower or equal to zero (the robot is turning left, right or is facing forward) and calls the function "turning" assigning to it a symbolic number corresponding to the observed state (1 for left turn, 2 for right turn, 3 for front facing) and also the current facing direction of the RGB-D camera (d).   
"""
def headturning(msg,client):
    check = rospy.get_param("/my_head_state")
    p = check['p']
    d = check['dir']
    if p == 1:
        if msg.angular.z > 0.0:
            turning(client,1,d)
        elif msg.angular.z < 0.0:
            turning(client,2,d)
        else:
            turning(client,3,d)
    else:
        print("head cannot move.")
"""
main function: 
It initializes the corresponding node, which acts both as a client and a subscriber. 

The point_head_action is needed to be able to turn the Tiago's head as we wish while it is in motion.

The cmd_vel topic is instead used to understand in which direction Tiago is going or turning, by checking the z-component of the published velocity.

This function also completes another task: it preventively lowers tiago's head, so that it reaches the right inclination to notice tables.  
"""  
def controller():
    rospy.init_node('my_head_control_client', anonymous=True)
    client = actionlib.SimpleActionClient('head_controller/point_head_action', PointHeadAction)
    client.wait_for_server()
    
    print("done operation: ",send_hcgoal(client,0.0,2*(0.456729/3),2))

    rospy.Subscriber("mobile_base_controller/cmd_vel", Twist, headturning, client)

    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass   


