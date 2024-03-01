import rclpy
from rclpy.node import Node
from motor_driver.canmotorlib import CanMotorController
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import pandas as pd
import subprocess
import platform
import os


def get_mac_address():
    
        mac_address = subprocess.check_output(f"cat /sys/class/net/wlan0/address", shell=True).decode().strip()
        
        if mac_address:
            return mac_address
            
        print("Error getting MAC address: No suitable interface found")
        return None

def find_robot_info(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, ['Species', 'Motor1_Direction', 'Motor2_Direction', 'Motor3_Direction']]
    if not match.empty:
        return match.iloc[0]
    else:
        return None


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('group_joint_controller')

        
        # Initialize publishers
        self.goat_publisher = self.create_publisher(JointState, "/Goat_joint_commands", 10)
        self.duck_publisher = self.create_publisher(JointState, "/Duck_joint_commands", 10)
        self.pony_publisher = self.create_publisher(JointState, "/Pony_joint_commands", 10)
        self.swan_publisher = self.create_publisher(JointState, "/Swan_joint_commands", 10)
        self.frog_publisher = self.create_publisher(JointState, "/Frog_joint_commands", 10)
        self.lion_publisher = self.create_publisher(JointState, "/Lion_joint_commands", 10)

        # Define motor command sequences
        self.motor_commands = [

            {'position': [20, 0, 0], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # First set of commands
            {'position': [20, 20, 0], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # Second set
            {'position': [20, 20, 20], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]}   # Third set
            
        ]

        # Initialize command index
        self.command_index = 0

        # Timer for sending commands every 4 seconds
        self.timer = self.create_timer(4.0, self.timer_callback)

    def timer_callback(self):

        if self.command_index < len(self.motor_commands):
            command = self.motor_commands[self.command_index]

            # Logging for debugging
            self.get_logger().info(f"Publishing command: {command}")

            # Ensure values are float
            position_command = [float(pos) for pos in command['position']]
            velocity_command = [float(vel) for vel in command['velocity']]
            effort_command = [float(eff) for eff in command['effort']]

            # Create and publish JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.position = position_command
            joint_state_msg.velocity = velocity_command
            joint_state_msg.effort = effort_command

            self.goat_publisher.publish(joint_state_msg)
            self.duck_publisher.publish(joint_state_msg)
            self.frog_publisher.publish(joint_state_msg)
            self.swan_publisher.publish(joint_state_msg)
            self.lion_publisher.publish(joint_state_msg)
            self.pony_publisher.publish(joint_state_msg)

            # Increment command index
            self.command_index += 1
        else:
            # Stop the timer if all commands have been sent
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()