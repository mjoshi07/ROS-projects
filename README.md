# ROS-projects
This repository contains some basic ROS projects


## Dependencies
* ROS (I have used ROS-Noetic but instructions are same for other versions as well)
* Ubuntu (20.04 for ROS Noetic)
* OpenCV (4.0.0 and above is supported by ROS Noetic, earlier versions of ROS support older versions of OpenCV) **[OPTIONAL]**

## Working
* Install ROS by following the guide: _http://wiki.ros.org/noetic/Installation_
* Create a ROS package: ``ros_basics`` using the following command 
```
catkin_create_pkg ros_basics std_msgs rospy roscpp
```
* Clone the repo inside the package and build it
```
catkin_make
```
* First start ROS master
```
roscore
```
* Run the different applications, as of now the following are available:
```
rosrun ros_basics talker_node
```
```
rosrun ros_basics listener_node
```

* Run the turtlesim application
```
rosrun turtlesim turtlesim_node
```
* TurtleBot motion applications:
```
rosrun ros_basics turtleBot_moveStraight
```
```
rosrun ros_basics turtleBot_rotate
```

```
rosrun ros_basics turtleBot_move2Location
```
```
rosrun ros_basics turtleBot_move2Home
```
```
rosrun ros_basics turtleBot_gridMotion
```
```
rosrun ros_basics turtleBot_spiralMotion
```
* ROS-OpenCV, publish and subscribe an image, can be used to stream(publish) videos as well
```
rosrun ros_basics image_publisher
```
```
rosrun ros_basics image_subscriber
```
```
rosrun ros_basics readVideo
```
* LaserScanner, scan laser data and print it
```
rosrun ros_basics scanSub
```
