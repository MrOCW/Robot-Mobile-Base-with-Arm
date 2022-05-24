## Mobile Robot with 6 DoF Arm
This project consists of code for SLAM, Autonomous Navigation, and 6 DoF Arm Manipulation.  

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