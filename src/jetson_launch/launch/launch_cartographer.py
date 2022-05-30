import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rs_prefix = get_package_share_directory('realsense2_camera')
   
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            rs_prefix,'launch','rs_launch.py'
        )])
    )
    lidar_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('jetson_launch'), 'launch','ydlidar.launch.py')
         ])
      )

    rtab_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('jetson_launch'), 'launch','rtabmap.launch.py')
         ])
      )

    urdf = get_package_share_directory('mobile_base_arm')+'/urdf/mobile_base_arm.urdf'
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf])

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        node_executable='joint_state_publisher'
    )

    serial_node = Node(
        package='ESP32_control',
        node_executable='ros2_serial',
        name='serial',
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        serial_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rs_launch,
        Node(package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='base_link_to_camera_link',
            arguments=['0.1083660968', '0.0475', '0.0501', '0', '0', '0', 'base_link', 'camera_link']),
        lidar_launch,
        #rtab_launch,
        
    ])
