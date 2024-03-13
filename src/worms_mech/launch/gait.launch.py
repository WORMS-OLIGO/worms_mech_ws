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
            # Launch the node with root access in a shell
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
        Node(
            package='worms_mech',
            executable='hardware_interface',
            namespace="",
            output="screen",
            name='hardware_interface',
            # Launch the node with root access in a shell
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
        
    ])