## Mobile Robot with 5 DoF Arm
This project consists of code for SLAM, Autonomous Navigation, and [5 DoF Arm](https://howtomechatronics.com/tutorials/arduino/diy-arduino-robot-arm-with-smartphone-control/) Manipulation.  

## Hardware
- NVIDIA Jetson Nano 4GB  
- ESP32  
- YDLidar X2L  
- DRV8833 (x2)  

## Software
- Ubuntu 18.04  
- ROS1 (Tested with Melodic)
- [MoveIt](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)  
- [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/)  


## Setup
```
$ git clone https://github.com/MrOCW/Robot-Mobile-Base-with-Arm && cd Robot-Mobile-Base-with-Arm
$ chmod +x setup.sh
$ ./setup.sh
$ source install_isolated/setup.bash
```
## Launch
Perform online SLAM with Cartographer
```
$ roslaunch cartographer_ros mobile_base_arm_slam.launch
```  

MoveIt on the arm
```
$ roslaunch moveit demo.launch
```  

Autonomous Navigation
```
$ roslaunch navigation navigation.launch
```

## Docker
```
$ sudo apt-get install python3-rocker
$ docker build -t <distro>-mobile-base-arm -f Dockerfile.ros.<distro> .
$ rocker <distro>-mobile-base-arm --x11 --nvidia --network host --privileged
$ source install_isolated/setup.bash
```