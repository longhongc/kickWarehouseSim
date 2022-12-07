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
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='robot_model').find('robot_model')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/office_bot_v5.urdf')
  robot_name_in_urdf = 'robot_model'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  world_file_name = 'smalltown.world'
  #world_file_name = 'office.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  #gui = LaunchConfiguration('gui')
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
    
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
        
  #declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    #name='gui',
    #default_value='True',
    #description='Flag to enable joint_state_publisher_gui')
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
    
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
    
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

 # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[default_model_path])

  # Publish the joint state values for the non-fixed joints in the URDF file.
  #start_joint_state_publisher_cmd = Node(
    #condition=UnlessCondition(gui),
    #package='joint_state_publisher',
    #executable='joint_state_publisher',
    #name='joint_state_publisher')

  # A GUI to manipulate the joint state values
  #start_joint_state_publisher_gui_node = Node(
    #condition=IfCondition(gui),
    #package='joint_state_publisher_gui',
    #executable='joint_state_publisher_gui',
    #name='joint_state_publisher_gui')

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  #start_robot_state_publisher_cmd = Node(
    #condition=IfCondition(use_robot_state_pub),
    #package='robot_state_publisher',
    #executable='robot_state_publisher',
    #parameters=[{'use_sim_time': use_sim_time, 
    #'robot_description': Command(['xacro ', model])}],
    #arguments=[default_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
    
  # include robot_localization local
  ekf_local_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory('robot_localization'),
              'launch/ekf_imu_odom_sim.launch.py'))
  )
  
  # include robot_localization global
  ekf_global_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory('robot_localization'),
              'launch/ekf_global_sim.launch.py'))
  )
  
  # include robot_localization navsat_transform
  navsat_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory('robot_localization'),
              'launch/navsat_transform_sim.launch.py'))
  )
  
  # include slam_toolbox
  #slam_toolbox_launch = IncludeLaunchDescription(
  #    PythonLaunchDescriptionSource(
  #        os.path.join(
  #            get_package_share_directory('slam_toolbox'),
  #            'launch/online_async_sim_launch.py'))
  #)
  
  # include nav2
  nav2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory('nav2_bringup'),
              'launch/navigation_gps_sim_launch.py'))
  )
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  #ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  #ld.add_action(start_joint_state_publisher_cmd)
  #ld.add_action(start_joint_state_publisher_gui_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(ekf_local_launch)
  ld.add_action(ekf_global_launch)
  ld.add_action(navsat_launch)
  #ld.add_action(slam_toolbox_launch)
  ld.add_action(nav2_launch)

  return ld
  
