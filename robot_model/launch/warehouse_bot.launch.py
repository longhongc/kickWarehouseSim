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
  ## Robot model
  robot_model_dir = FindPackageShare(package='robot_model').find('robot_model')
  robot_file_name = 'office_bot_v6.urdf'
  robot_file_path = os.path.join(robot_model_dir, 'models', robot_file_name)

  ## Robot localization
  ekf_file_name = 'ekf_test.yaml'
  ekf_params_path = os.path.join(robot_model_dir, 'params', ekf_file_name)

  ## Maps
  # map_path = os.path.join(robot_model_dir, 'maps', 'map_1670396336.yaml')
  # map_path = os.path.join(robot_model_dir, 'maps', 'map_2.yaml')
  # map_path = os.path.join(robot_model_dir, 'maps', 'test.yaml')
  map_path = os.path.join(robot_model_dir, 'maps', 'warehouse_map.yaml')
  serialized_map_path = os.path.join(robot_model_dir, 'maps', 'warehouse_map_serialized')

  ## Slam toolbox
  slam_toolbox_file_name = 'mapper_params_online_async.yaml'
  slam_toolbox_params_path = os.path.join(robot_model_dir, 'params', 'slam_toolbox', slam_toolbox_file_name)

  ## Nav2
  nav2_bringup_dir = get_package_share_directory('nav2_bringup')
  nav2_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),

  # Launch configuration variables specific to simulation
  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  model = LaunchConfiguration('model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_robot_localization = LaunchConfiguration('use_robot_localization')

  ## Nav2 launch configuration
  slam = LaunchConfiguration('slam')
  autostart = LaunchConfiguration('autostart')

  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')

  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=robot_file_path, 
    description='Absolute path to robot urdf file')
    
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

  declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

  declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

  # Specify the actions

  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
 
  ## Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace=namespace,
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time}],
    remappings=remappings,
    arguments=[robot_file_path])
      
  # include robot_localization
  # start_ekf_cmd = Node(
  #   package='robot_localization',
  #   executable='ekf_node',
  #   name='ekf_filter_node',
  #   output='screen',
  #   parameters=[ekf_params_path],
  #   condition=IfCondition(use_robot_localization))

  ## include slam_toolbox
  start_slam_toolbox_cmd = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[slam_toolbox_params_path,
         {'use_sim_time': use_sim_time,
          'map_start_at_dock': True,
          'map_file_name': serialized_map_path}])

    # executable='sync_slam_toolbox_node',
    # executable='async_slam_toolbox_node',
    # executable='localization_slam_toolbox_node',

  start_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_path,
                          'params_file': nav2_params_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart}.items())

  # include nav2
  # nav2_launch = IncludeLaunchDescription(
  #    PythonLaunchDescriptionSource(
  #        os.path.join(
  #            get_package_share_directory('nav2_bringup'),
  #            'launch/navigation_sim_launch.py'))
  # )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_robot_localization_cmd)

  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_autostart_cmd)

  # Add any actions
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_ekf_cmd)
  ld.add_action(start_slam_toolbox_cmd)
  # ld.add_action(nav2_launch)
  ld.add_action(start_nav2_bringup_cmd)

  return ld
  
