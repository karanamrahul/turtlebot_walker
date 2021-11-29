


[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project
# Turtlebot_walker
A walker algorithm for turtlebot3 to avoid obstacles.It modifies the usual turtlebot3 to perform obstacle avoidance by sensing a obstacle and rotating when there is a obstacle, else move forward.



## Dependencies

The following dependencies are required to run this package:

1. ROS noetic
2. catkin 
3. [Ubuntu 20.04 For installing ROS](http://wiki.ros.org/noetic)
4. [turtlebot3_simulation packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)


## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/karanamrahul/turtlebot_walker
cd ..
catkin_make
```

Open the following terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
Passing record=true for recording of bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker tb3_roomba.launch record:=true
```

Passing record=false for not recording the bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker tb3_roomba.launch record:=false
```


## Print information in a bag file
1. Terminal 1:
```
cd catkin_ws
source devel/setup.bash
cd src/turtlebot_walker/results/
rosbag info roomba.bag
```

## Playing a rosbag file
1. Terminal 1 (run master node):
```
roscore
```

2. Terminal 2 ([play rosbag](https://drive.google.com/file/d/14nKEnZNMQerkhuANQ8l0ABdd5cKx7ffr/view?usp=sharing)):
```
cd catkin_ws
source devel/setup.bash
rosbag play src/turtlebot_walker/results/roomba.bag
```
