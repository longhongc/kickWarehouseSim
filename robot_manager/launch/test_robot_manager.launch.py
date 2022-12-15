from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    test_robot_manager_alone = Node(
       package='robot_manager',
       executable='robot_manager_test',
       name='robot_manager_test_node',
       output='screen',
       emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(test_robot_manager_alone)

    return ld
