echo "System is running Ubuntu $(lsb_release -rs)"

if [[ $(lsb_release -rs) == "18.04" ]]
then
    ROS_DISTRO=melodic
elif [[ $(lsb_release -rs) == "20.04" ]]
then
    ROS_DISTRO=noetic
fi
echo -e "Install ROS \033[0;31m${ROS_DISTRO}\033[0m and the respective dependencies? [y/n]"

if [[ $1 != "-y" ]]
then
    read install
else
    install="y"
fi

if [[ ${install} == "y" ]]
then
    # Install rosserial
    sudo apt-get install -y ros-${ROS_DISTRO}-rosserial-arduino
    sudo apt-get install -y ros-${ROS_DISTRO}-rosserial
    cd src/ESP32_control/lib
    rosrun rosserial_arduino make_libraries.py .

    # Install teleop twist keyboard
    sudo apt-get install ros-${ROS_DISTRO}-teleop-twist-keyboard

    # Install cartographer
    cd ../../..
    if [[ ${ROS_DISTRO} == melodic ]]
    then
        sudo apt-get install -y python-wstool python-rosdep ninja-build stow
    elif [[ ${ROS_DISTRO} == noetic ]]
    then
        sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
    fi
    wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src
    src/cartographer/scripts/install_abseil.sh
    sudo mv src/cartographer_files/mobile_base_arm.lua src/cartographer_ros/cartographer_ros/configuration_files/mobile_base_arm.lua
    sudo mv src/cartographer_files/mobile_base_arm.rviz src/cartographer_ros/cartographer_ros/configuration_files/mobile_base_arm.rviz
    sudo mv src/cartographer_files/mobile_base_arm_slam.launch src/cartographer_ros/cartographer_ros/launch/mobile_base_arm_slam.launch
    sudo rm -rf src/cartographer_files

    sudo apt-get install -y ros-${ROS_DISTRO}-imu-tools 

    # Install Moveit
    #sudo apt-get update
    #sudo apt-get dist-upgrade
    if [[ ${ROS_DISTRO} == melodic ]]
    then
        sudo apt-get install -y ros-${ROS_DISTRO}-catkin python-catkin-tools
        sudo apt install -y ros-${ROS_DISTRO}-moveit
    elif [[ ${ROS_DISTRO} == noetic ]]
    then
        sudo apt install -y ros-${ROS_DISTRO}-catkin python3-catkin-tools python3-osrf-pycommon
    fi
    sudo apt-get install -y ros-${ROS_DISTRO}-navigation

    # Install ydlidar package
    cd src
    sudo rm -rf ydlidar_ros
    git clone https://github.com/YDLIDAR/ydlidar_ros
    

    sudo rm -rf realsense-ros
    git clone https://github.com/IntelRealSense/realsense-ros.git
    sudo mv realsense-files/imu_optical_to_ros.cpp realsense-ros/realsense2_camera/src/imu_optical_to_ros.cpp
    sudo mv realsense-files/rs_camera.launch realsense-ros/realsense2_camera/launch/rs_camera.launch
    sudo mv realsense-files/CMakeLists.txt realsense-ros/realsense2_camera/CMakeLists.txt
    sudo rm -rf realsense-files

    sudo rm -rf rtabmap_ros
    git clone https://github.com/introlab/rtabmap_ros
    #sudo mv rtabmap-files/rgbd_vo.launch rtabmap_ros/launch/rgbd_vo.launch
    #sudo mv rtabmap-files/CMakeLists.txt rtabmap_ros/CMakeLists.txt
    #sudo rm -rf rtabmap-files
    
    cd ..
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    
    catkin_make_isolated -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release --install --use-ninja
    echo "Setup complete! Please edit ESP32_Control/lib/ros_lib/ros.h to include \"ArduinoHardware.h\" if using UART/Serial"
else
    echo "Terminated"
fi

