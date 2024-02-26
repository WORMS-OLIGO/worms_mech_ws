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
    
    mac_address = subprocess.check_output(f"cat /sys/class/net/wlan0/address", shell=True).decode().strip()
    
    if mac_address:
        return mac_address
        
    print("Error getting MAC address: No suitable interface found")
    return None

def find_robot_name(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    print(df)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('gait_manager')


        # Construct the path to the CSV file
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        species = find_robot_name(mac_address, spreadsheet_path)

        if species is None:
            raise ValueError("Robot species not found. Please check the MAC address and spreadsheet.")

        joint_commands_topic = f'/{species}_joint_commands'
        joint_states_topic = f'/{species}_joint_states'
        worm_action = f'/{species}_action'


        self.command_publisher = self.create_publisher(JointState, joint_commands_topic, 10)

        self.state_subscriber = self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)

        self.action_subscriber = self.create_subscription(String, worm_action, self.actions_callback, 10)

        
        self.execute_timer_callback = False

        # WAYPOINTS FOR EACH DISCRETE GAIT ACTION
        self.br_step_waypoints = [
            
            [0, 145, 175], # LIFTED LEG POSITION WHEN ON THE FLOOR
            [15, 145, 175], # TAKING STEP WITH SHOE ELEVATED - 15 DEGREE MOTION
            [15, 135, 175]  # BRING SHOE DOWN WHEN ON FLOOR
        ]

        self.br_prone_waypoints = [
            [0, -120, 175]  
        ]

        self.br_lift_waypoints = [
            [0, 0, 0],
            [15, -45, 35]
        ]

        # INTERPOLATE DISCRETE ACTIONS
        self.br_step_interpolated_positions = self.interpolate_waypoints(self.br_step_waypoints, .5)
        self.br_prone_interpolated_positions = self.interpolate_waypoints(self.br_prone_waypoints, .5)
        self.br_lift_interpolated_positions = self.interpolate_waypoints(self.br_lift_waypoints, .5)


        self.position_index = 0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def joint_state_callback(self, msg):
        
        # Update the first waypoint with the current position
        self.current_pose = msg

        if hasattr(self, 'current_pose') and self.current_pose is not None:
            position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
            self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')
            self.current_position = self.current_pose.position


    def interpolate_waypoints(self, waypoints, interval_degrees=1):
        interpolated = [self.current_position]
        for waypoint in waypoints:
            distance = np.linalg.norm(np.array(waypoint) - np.array(interpolated[-1]))
            steps = int(distance / interval_degrees)
            for step in range(1, steps + 1):
                interpolated_point = interpolated[-1] + (np.array(waypoint) - np.array(interpolated[-1])) * (step / steps)
                interpolated.append(interpolated_point)
        return interpolated

    def timer_callback(self):

        if self.execute_timer_callback:
            
            if self.position_index < len(self.interpolated_positions):
                # Ensure the position command is a list of floats
                self.position_command = list(map(float, self.interpolated_positions[self.position_index]))

                self.get_logger().info(f"Publishing command: {self.position_command}")

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
                joint_state_msg.effort = [0.0, 0.0, 0.0]
                print(joint_state_msg)
                self.publisher.publish(joint_state_msg)


                self.position_index += 1

            else:
                

                joint_state_msg = JointState()

                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
                joint_state_msg.effort = [0.0, 0.0, 0.0]
                
                if hasattr(self, 'current_pose') and self.current_pose is not None:
                    position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
                    self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')

                self.publisher.publish(joint_state_msg)
        else:
            self.execute_timer_callback = False
            

    def actions_callback(self, msg):
        if msg.data == "step":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(br_step_waypoints)
            self.action = "step"

        if msg.data == "lift":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(br_lift_waypoints)
            self.action = "lift"

        if msg.data == "prone":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(br_prone_waypoints)
            self.action = "prone"

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


