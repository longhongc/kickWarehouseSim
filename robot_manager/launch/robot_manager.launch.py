import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_manager_dir = FindPackageShare(package='robot_manager').find('robot_manager')
    config_file_name = 'waypoints.yaml'
    config_path = os.path.join(robot_manager_dir, 'config', config_file_name)

    # Launch configuration variables specific to simulation
    namespace = LaunchConfiguration('namespace')
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_config_cmd = DeclareLaunchArgument(
        name='config',
        default_value=config_path,
        description='Configuration for robot mission setting such as waypoints')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # Specify the actions
    start_robot_manager_cmd = Node(
        package='robot_manager',
        executable='robot_manager',
        namespace=namespace,
        name='robot_manager',
        output='screen',
        parameters=[config,
                    {'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_robot_manager_cmd)

    return ld
