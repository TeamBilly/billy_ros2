import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription();

    config_billy_joy = os.path.join(
        get_package_share_directory('billy_description'),
        'config',
        'billy_teleop_default.yaml'
    )

    billy_teleop_node = Node(
        package='billy_teleop',
        executable='joy_manager',
        name='billy_teleop_node',
        emulate_tty=True,
        output='screen',
        parameters = [config_billy_joy]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node'
    )

    ld.add_action(joy_node)
    ld.add_action(billy_teleop_node)

    return ld
