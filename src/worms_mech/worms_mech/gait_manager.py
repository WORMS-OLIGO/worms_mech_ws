import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import pandas as pd
import subprocess
import platform
import os

def get_mac_address():
    # This command should work on Linux
    try:
        # Tries to get the MAC address of the first network interface
        interface = subprocess.check_output("ls /sys/class/net | head -n 1", shell=True).decode().strip()
        mac_address = subprocess.check_output(f"cat /sys/class/net/{interface}/address", shell=True).decode().strip()
        return mac_address
    except Exception as e:
        print(f"Error getting MAC address: {e}")
        return None

def find_robot_name(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('gait_manager')

        # Move MAC address retrieval and robot name finding into the class initializer

      

        # Construct the path to the CSV file
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        print("Mac Address: " + mac_address)
        species = find_robot_name(mac_address, spreadsheet_path)

        if species is None:
            raise ValueError("Robot species not found. Please check the MAC address and spreadsheet.")

        joint_commands_topic = f'/{species}_joint_commands'
        joint_states_topic = f'/{species}_joint_states'

        print(joint_commands_topic)
        print(joint_states_topic)

        # self.publisher = self.create_publisher(JointState, joint_commands_topic, 10)
        # self.subscription = self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)

        # self.waypoints = [
        #     [0, 0, 0], [0, -170, 115], [30, -170, 115], [30, -45, 45], [0, -45, 45], [0, 0, 0]
        # ]

        # self.interpolated_positions = self.interpolate_waypoints(self.waypoints, .5)
        # self.position_index = 0
        # self.timer = self.create_timer(0.1, self.timer_callback)

    # Remaining class methods unchanged...

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


