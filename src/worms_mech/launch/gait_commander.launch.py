from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    hardware_interface_node = Node(
        package="worms_mech",
        executable="hardware_interface",
    )

    gait_manager = Node(
        package="worms_mech",
        executable="gait_manager",
    )

    action_node = Node(
        package="worms_mech",
        executable="action_node",
    )

    ld.add_action(hardware_interface_node)
    ld.add_action(gait_manager_node)
    ld.add_action(action_node)
    return ld