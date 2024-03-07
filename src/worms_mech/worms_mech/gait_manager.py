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
            head = file.readline().strip() 

        print("HEAD CONNECTOR: " + head)


        #print("WORM_ID: " + worm_id)

        configuration_info = find_species(head,specialization_path)

        print(configuration_info)

        """
        These variables define the direction of the motors that are specific to the current gait assembled
        In the case of our 4 legged system, we use 1 and -1 to define which side each worm would be on in the 
        standard forward operating condition, which makes:
        ------------------------
        1 = Left Side Leg
        -1 = Right Side Leg
        ------------------------
        The Effect of the -1 on the Right side is to flip the direction of the head motor when executing 
        the same step command

        """


        self.motor1_side_orientation = 1
        self.motor2_side_orientation = 1
        self.motor3_side_orientation = 1

        self.species = head

        mac_address = get_mac_address()

        worm_id = find_robot_name(mac_address, spreadsheet_path)

       
        self.motor1_side_orientation = configuration_info['Motor1_Direction']
        self.motor2_side_orientation = configuration_info['Motor2_Direction']
        self.motor3_side_orientation = configuration_info['Motor3_Direction']
 


        # if self.species is None:
        #     raise ValueError("Robot species not found. Please check the camera node and text file created.")

        joint_commands_topic = f'/{worm_id}_joint_commands'
        joint_states_topic = f'/{worm_id}_joint_states'
        coordination_topic = f'/{worm_id}_coordination'
        worm_action = 'actions'

        print("Sending Commands To: " + joint_commands_topic)
        print("Getting Joint States From: " + joint_states_topic)
        

        self.command_publisher = self.create_publisher(JointState, joint_commands_topic, 10)

        self.coordination_publisher = self.create_publisher(String, coordination_topic, 10)

        self.state_subscriber = self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)
        self.action_subscriber = self.create_subscription(String, worm_action, self.actions_callback, 10)

        
        self.execute_timer_callback = False


        # 1 = Robot moving forward with BEAR (Front Left) and BIRD (Front Right) in the direction of motion 
        # -1 = Robot Moving Backwards with BULL (Front Left) and BOAR (Front Right) leading in the direction of motion
        self.forward_mode = 1

        

        # WAYPOINTS FOR EACH DISCRETE GAIT ACTION
        self.stand_step_waypoints = [
            [0, 150, -120], # LIFTED LEG POSITION WHEN ON THE FLOOR
            [20, 160, -120], # TAKING STEP WITH SHOE ELEVATED - 15 DEGREE MOTION
            [20, 135, -120]  # BRING SHOE DOWN WHEN ON FLOOR
        ]

        self.stand_prone_waypoints = [
            [0, 150, -120]  
        ]

        self.stand_stand_waypoints = [
            [20, 45, -35]
        ]

        self.stand_test_gait_waypoints = [
            [20, 140, -90],
        ]

        self.stand_minimal_motion_waypoints = [
            [10, 10, 10]  
        ]

        self.stand_propel_waypoints = [
            [0, 45, -35]
        ]

        self.stand_forward_gait = [
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120]
        ]

        self.stand_reverse_gait = [
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120]
        ]

        self.stand_forward_reverse_gait = [

            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [-20, 160, -120],
            [-20, 135, -120],
            [-20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120],
            [20, 160, -120],
            [20, 135, -120],
            [20, 45, -35],
            [0, 45, -35],
            [0, 150, -120]

        ]


#==================
        self.field_step_waypoints = [
            [0, 20, 0], # LIFTED LEG POSITION WHEN ON THE FLOOR
            [20, 0, 0], # TAKING STEP WITH SHOE ELEVATED - 15 DEGREE MOTION
            [20, -15, 0]  # BRING SHOE DOWN WHEN ON FLOOR
        ]

        self.field_prone_waypoints = [
            [0, 0, 0]
        ]

        self.field_stand_waypoints = [
            [20, -195, 85]
        ]

        self.field_test_gait_waypoints = [
            [20, -10, 30],
        ]

        self.field_propel_waypoints = [
            [0, -195, 85]
        ]

        
        self.position_index = 0
        self.current_position = [0, 0, 0]

        self.timer = self.create_timer(0.02, self.timer_callback)

    def joint_state_callback(self, msg):
        
        # Update the first waypoint with the current position
        self.current_pose = msg

        if hasattr(self, 'current_pose') and self.current_pose is not None:
            position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
            self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')
            self.current_position = self.current_pose.position


    def interpolate_waypoints(self, arrays, increment=0.3):
        """
        Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
        lists representing each incremental step towards the waypoints.

        :param arrays: A sequence of arrays where each array is a list of numbers.
        :param increment: The incremental value to adjust the numbers. Default is 0.1.
        :return: A list of lists representing each step through the waypoints.
        """
        transition_steps = []

        # Start from the current position
        current_state = self.current_position

        for target_array in arrays:
            while True:
                step = []
                done = True
                for current_val, target_val in zip(current_state, target_array):
                    if abs(target_val - current_val) > increment:
                        done = False
                        if target_val > current_val:
                            step.append(current_val + increment)
                        else:
                            step.append(current_val - increment)
                    else:
                        step.append(target_val)
                
                current_state = step
                transition_steps.append(step)
                
                if done:
                    break

        return transition_steps

    def timer_callback(self):

        msg = String ()
        msg.data = "Static"

        # Flag thats set to true when the worm receives some action command (ex: step, prone, walk etc.)
        if self.execute_timer_callback:

            msg.data = "in_progress"
            
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

                msg.data = "done"

                self.position_index = 0
                self.execute_timer_callback = False

                joint_state_msg = JointState()

                joint_state_msg.position = self.position_command
                joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
                joint_state_msg.effort = [0.0, 0.0, 0.0]
                
                if hasattr(self, 'current_pose') and self.current_pose is not None:
                    position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
                    self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')

                self.command_publisher.publish(joint_state_msg)
        
            

        self.coordination_publisher.publish(msg)
            

    def actions_callback(self, msg):
        if msg.data == "stand_step":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_step_waypoints)
            self.action = "stand_step"
            self.execute_timer_callback = True

        if msg.data == "stand_stand":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_stand_waypoints)
            self.action = "stand_stand"
            self.execute_timer_callback = True

        if msg.data == "stand_prone":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_prone_waypoints)
            self.action = "stand_prone"
            self.execute_timer_callback = True

        if msg.data == "stand_test_gait":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_test_gait_waypoints)
            self.action = "stand_test_gait"
            self.execute_timer_callback = True

        if msg.data == "stand_min_test":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_minimal_motion_waypoints)
            self.action = "stand_minimal_test"
            self.execute_timer_callback = True

        if msg.data == "stand_propel":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_propel_waypoints)
            self.action = "stand_propel"
            self.execute_timer_callback = True

        if msg.data == "stand_forward_gait":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_forward_gait)
            self.action = "stand_forward_gait"
            self.execute_timer_callback = True

        if msg.data == "stand_reverse_gait":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_reverse_gait)
            self.action = "stand_reverse_gait"
            self.execute_timer_callback = True

        if msg.data == "stand_forward_reverse_gait":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.stand_forward_reverse_gait)
            self.action = "stand_forward_reverse_gait"
            self.execute_timer_callback = True

        #=============================================================
        if msg.data == "field_step":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.field_step_waypoints)
            self.action = "field_step"
            self.execute_timer_callback = True

        if msg.data == "field_stand":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.field_stand_waypoints)
            self.action = "field_stand"
            self.execute_timer_callback = True

        if msg.data == "field_prone":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.field_prone_waypoints)
            self.action = "field_prone"
            self.execute_timer_callback = True


        if msg.data == "field_test_gait":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.field_test_gait_waypoints)
            self.action = "field_test_gait"
            self.execute_timer_callback = True

        if msg.data == "field_propel":
            # Interpolate Positions from Current Position to Start of the Desired Action
            self.interpolated_positions = self.interpolate_waypoints(self.field_propel_waypoints)
            self.action = "field_propel"
            self.execute_timer_callback = True

            

        if msg.data == "reverse":
            self.action = "reverse"
            self.forward_mode = -1
            self.execute_timer_callback = True

        if msg.data == "forward":
            self.action = "forward"
            self.forward_mode = 1
            self.execute_timer_callback = True

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    
    rclpy.spin(joint_command_publisher)


    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


