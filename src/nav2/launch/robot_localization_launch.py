import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

  imu_filter_node = launch_ros.actions.Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    name='imu_filter_madgwick',
    output='screen',
    parameters=[{
                "use_mag": False,
            }],
    remappings=[('/imu/data_raw', '/camera/imu_ros'),]
  )
  robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('nav2'), 'param/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                        description='Flag to enable use_sim_time'),
    imu_filter_node,
    robot_localization_node,
  ])
