from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='worms_mech',
            executable='gait_manager',
            namespace="",
            output="screen",
            name='gait_manager',
            shell=True,
        ),
        Node(
            package='worms_mech',
            executable='hardware_interface',
            namespace="",
            output="screen",
            name='hardware_interface',
            shell=True,
        ),
        
    ])