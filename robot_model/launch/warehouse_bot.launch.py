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
  pkg_share = FindPackageShare(package='robot_model').find('robot_model')
  robot_file_name = 'office_bot_v5.urdf'
  robot_file_path = os.path.join(pkg_share, 'models', robot_file_name)
 
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config_with_model.rviz')

  ekf_file_name = 'ekf_test.yaml'
  ekf_params_path = os.path.join(pkg_share, 'params', ekf_file_name)
  
  # Launch configuration variables specific to simulation
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_robot_localization = LaunchConfiguration('use_robot_localization')
  
  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=robot_file_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
 
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
      
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_use_robot_localization_cmd = DeclareLaunchArgument(
    name='use_robot_localization',
    default_value='False',
    description='Whether to use robot localization package')

  # Specify the actions

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[robot_file_path])
      
  # include robot_localization
  start_ekf_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_params_path],
    condition=IfCondition(use_robot_localization))

  # include slam_toolbox
  #slam_toolbox_launch = IncludeLaunchDescription(
  #    PythonLaunchDescriptionSource(
  #        os.path.join(
  #            get_package_share_directory('slam_toolbox'),
  #            'launch/online_async_sim_launch.py'))
  #)
  
  # include nav2
  #nav2_launch = IncludeLaunchDescription(
  #    PythonLaunchDescriptionSource(
  #        os.path.join(
  #            get_package_share_directory('nav2_bringup'),
  #            'launch/navigation_sim_launch.py'))
  #)
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_robot_localization_cmd)

  # Add any actions
  ld.add_action(start_rviz_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_ekf_cmd)
  #ld.add_action(slam_toolbox_launch)
  #ld.add_action(nav2_launch)

  return ld
  
