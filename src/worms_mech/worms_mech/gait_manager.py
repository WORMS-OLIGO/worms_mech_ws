import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None

def find_species(head, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['Head'] == head, ['Specialization', 'Motor1_Direction', 'Motor2_Direction', 'Motor3_Direction']]
    if not match.empty:
        return match.iloc[0]
    else:
        return None

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('gait_manager')


        # Construct the path to the CSV file for worm info
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        # Construct the path to the CSV file that holds specialization data
        specialization_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/specialization_table.csv')

        # Define the path to the file that contains head information from camera node
        head_connection_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/head.txt')

        # Open the file and read its contents
        with open(head_connection_path, 'r') as file:
            head = file.read()

        print("HEAD CONNECTOR: " + head)


        #print("WORM_ID: " + worm_id)

        configuration_info = find_species(head,specialization_path)

        print(configuration_info)

        # These variables define the direction of the motors that are specific to the current gait assembled
        # In the case of our 4 legged system, we use 1 and -1 to define which side each worm would be on in the 
        # standard forward operating condition, which makes:
        # ------------------------
        # 1 = Left Side Leg
        # -1 = Right Side Leg
        # ------------------------
        # The Effect of the -1 on the Right side is to flip the direction of the head motor when executing 
        # the same step command

        self.motor1_side_orientation = 1
        self.motor2_side_orientation = 1
        self.motor3_side_orientation = 1

        self.species = head

        mac_address = get_mac_address()

        worm_id = find_robot_name(mac_address, spreadsheet_path)

        # # Check if worm_info is not None
        # if configuration_info is not None:
        #     self.species = head
        #     self.motor1_side_orientation = configuration_info['Motor1_Direction']
        #     self.motor2_side_orientation = configuration_info['Motor2_Direction']
        #     self.motor3_side_orientation = configuration_info['Motor3_Direction']

        #     #print(f"{worm_id} Has Been Connected to Accessory Port {self.species}")
        #     print(f"Motor Direction 1: {self.motor1_side_orientation}")
        #     print(f"Motor Direction 2: {self.motor2_side_orientation}")
        #     print(f"Motor Direction 3: {self.motor3_side_orientation}")
        # else:
        #    # print("No configuration found for the given head connection.")


        # if self.species is None:
        #     raise ValueError("Robot species not found. Please check the camera node and text file created.")

        joint_commands_topic = f'/{worm_id}_joint_commands'
        joint_states_topic = f'/{worm_id}_joint_states'
        worm_action = 'actions'

        print("Sending Commands To: " + joint_commands_topic)
        print("Getting Joint States From: " + joint_states_topic)
        

        self.command_publisher = self.create_publisher(JointState, joint_commands_topic, 10)
        self.coordination_publisher = self.create_publisher(String, "/coordination", 10)
        self.state_subscriber = self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)
        self.action_subscriber = self.create_subscription(String, worm_action, self.actions_callback, 10)

        
        self.execute_timer_callback = False


        # 1 = Robot moving forward with BEAR (Front Left) and BIRD (Front Right) in the direction of motion 
        # -1 = Robot Moving Backwards with BULL (Front Left) and BOAR (Front Right) leading in the direction of motion
        self.forward_mode = 1

        

        # WAYPOINTS FOR EACH DISCRETE GAIT ACTION
        self.step_waypoints = [
            [0, 145, 175], # LIFTED LEG POSITION WHEN ON THE FLOOR
            [15, 145, 175], # TAKING STEP WITH SHOE ELEVATED - 15 DEGREE MOTION
            [15, 135, 175]  # BRING SHOE DOWN WHEN ON FLOOR
        ]

        self.prone_waypoints = [
            [0, 150, -120]  
        ]

        self.stand_waypoints = [
            [15, -45, 35]
        ]

        self.test_gait_waypoints = [
            [20, 120, -90],
        ]

        
        self.position_index = 0
        self.current_position = [0, 0, 0]

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

        # Flag thats set to true when the worm receives some action command (ex: step, prone, walk etc.)
        if self.execute_timer_callback:
            
            # Goes through all of the position that are contained within the trajectory created for that specific action
            if self.position_index < len(self.interpolated_positions):

                # Ensure the position command is a list of floats
                self.position_command = list(map(float, self.interpolated_positions[self.position_index]))

                self.get_logger().info(f"Publishing command: {self.position_command}")

                # POSITION_COMMAND[0] IS GETTING SENT TO HIP
                self.position_command[0] *= self.motor1_side_orientation * self.forward_mode    #Only sending to the head in this specific 4 legged walker configuration
                self.position_command[1] *= self.motor2_side_orientation
                self.position_command[2] *= self.motor3_side_orientation

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
                joint_state_msg.effort = [0.0, 0.0, 0.0]
                print(joint_state_msg)
                self.command_publisher.publish(joint_state_msg)


                self.position_index += 1

            else:

                joint_state_msg = JointState()

                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
                joint_state_msg.effort = [0.0, 0.0, 0.0]
                
                if hasattr(self, 'current_pose') and self.current_pose is not None:
                    position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
                    self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')

                self.command_publisher.publish(joint_state_msg)
        else:
            msg = String ()
            msg.data = "done"
            self.coordination_publisher.publish(msg)
            self.execute_timer_callback = False
            

    def actions_callback(self, msg):
        if msg.data == "step":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.step_waypoints)
            self.action = "step"

        if msg.data == "stand":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_waypoints)
            self.action = "stand"

        if msg.data == "prone":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.prone_waypoints)
            self.action = "prone"


        if msg.data == "test_gait":
            self.execute_timer_callback = True

            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.test_gait_waypoints)
            self.action = "test_gait"

        if msg.data == "reverse":
            self.action = "reverse"
            self.forward_mode = -1

        if msg.data == "forward":
            self.action = "forward"
            self.forward_mode = 1

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


