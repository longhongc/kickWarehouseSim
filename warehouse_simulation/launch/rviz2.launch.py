import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():

  # Set the path to different files and folders.
  warehouse_simulation_dir = FindPackageShare(package='warehouse_simulation').find('warehouse_simulation')
  rviz_file_name = 'mapping_and_navigation.rviz'
  default_rviz_config_path = os.path.join(warehouse_simulation_dir, 'rviz', rviz_file_name)

  rviz_config_file = LaunchConfiguration('rviz_config_file')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_rviz_config_file_cmd)

  # Add any actions
  ld.add_action(start_rviz_cmd)

  return ld
