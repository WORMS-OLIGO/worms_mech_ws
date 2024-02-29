import rclpy
from rclpy.node import Node
from motor_driver.canmotorlib import CanMotorController
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


class MotorControllerNode(Node):
            

    def __init__(self):
        super().__init__('hardware_interface')

        # Construct the path to the CSV file
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        worm_info = find_robot_info(mac_address, spreadsheet_path)

        # Check if worm_info is not None
        if worm_info is not None:
            self.worm_id = worm_info['Species']
            self.motor1_direction = worm_info['Motor1_Direction']
            self.motor2_direction = worm_info['Motor2_Direction']
            self.motor3_direction = worm_info['Motor3_Direction']

            print(f"{self.worm_id} Has Been Initialized")
            print(f"Motor Direction 1: {self.motor1_direction}")
            print(f"Motor Direction 2: {self.motor2_direction}")
            print(f"Motor Direction 3: {self.motor3_direction}")
        else:
            print("No matching robot found for the given MAC address.")

        self.joint_commands_topic = f'{self.worm_id}_joint_commands'
        self.joint_states_topic = f'{self.worm_id}_joint_states'
        self.worm_heartbeat_topic = f'{self.worm_id}_heartbeat'

        print("Recieving Commands From: " + self.joint_commands_topic)
        print("Joint States Publishing To: " + self.joint_states_topic)

        
        self.motor_controller_dict = {}

        self.worm_heartbeat = None
        
        self.head_connection = None
        self.tail_connection = None
        
        # Hardcoded to match pican board
        can_device = 'can0'
        motor_ids = [1, 2, 3]

        self.pos1 = 0
        self.pos2 = 0
        self.pos3 = 0

        for motor_id in motor_ids:
            self.motor_controller_dict[motor_id] = CanMotorController(can_device, motor_id, motor_type="AK80_6_V2")

        print("Creating Subscriber")
        self.subscription = self.create_subscription(
            JointState,
            self.joint_commands_topic,
            self.joint_commands_callback,
            10)

        self.publisher = self.create_publisher(JointState, self.joint_states_topic, 10)

        self.heartbeat_publisher = self.create_publisher(String, self.worm_heartbeat_topic, 10)

        self.worm_heartbeat = String()
        self.worm_heartbeat.data = "Disabled"

        self.get_logger().info("Enabling Motors...")

        for motor_id, motor_controller in self.motor_controller_dict.items():
            state = motor_controller.enable_motor()

            if(state == None):
                self.worm_heartbeat.data = "Disabled"
            else:
                self.worm_heartbeat.data = "Enabled"


    def joint_commands_callback(self, msg):
        # Handle the incoming joint state command messages here
        
        # Assuming the JointState message will have as many entries as there are motors
        for idx, (motor_id, motor_controller) in enumerate(self.motor_controller_dict.items()):
            # Assuming Kp and Kd as constants for this example
            Kp = 45
            Kd = 2

            joint_state_msg = JointState()

            vel_command = msg.velocity[idx]
            K_ff = msg.effort[idx]
            pos_command = msg.position[idx]


            if(motor_id == 1 and (abs((self.pos1 * self.motor1_direction) - (pos_command * self.motor1_direction)) < 10)):
                    self.pos1, self.vel1, self.curr1 = self.motor1_direction * motor_controller.send_deg_command(pos_command * self.motor1_direction, vel_command, Kp, Kd, K_ff)
                    
                    if(self.pos1 == None):
                        self.worm_heartbeat.data = "Disabled"
                    else: 
                        self.worm_heartbeat.data = "Enabled"

            elif(motor_id == 2  and (abs((self.pos2 * self.motor2_direction) - (pos_command * self.motor2_direction)) < 10)):
                    self.pos2, self.vel2, self.curr2 = self.motor2_direction * motor_controller.send_deg_command(pos_command * self.motor2_direction, vel_command, Kp, Kd, K_ff)
                    
                    if(self.pos1 == None):
                        self.worm_heartbeat.data = "Disabled"
                    else: 
                        self.worm_heartbeat.data = "Enabled"

            elif(motor_id == 3  and (abs((self.pos3 * self.motor3_direction) - (pos_command * self.motor3_direction)) < 10)):
                    self.pos3, self.vel3, self.curr3 = self.motor3_direction * motor_controller.send_deg_command(pos_command * self.motor3_direction, vel_command, Kp, Kd, K_ff)

                    if(self.pos1 == None):
                        self.worm_heartbeat.data = "Disabled"
                    else: 
                        self.worm_heartbeat.data = "Enabled"

            elif(abs((self.pos3 * self.motor3_direction) - (pos_command * self.motor3_direction)) > 10):
                print("Commanded Position is Over Threshold. Value is: " + str(abs((self.pos1 * self.motor1_direction) - (pos_command * self.motor1_direction))))

           

        
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["motor_1", "motor_2", "motor_3"]
        joint_state_msg.position = [self.pos1, self.pos2, self.pos3]
        joint_state_msg.velocity = [self.vel1, self.vel2, self.vel3]
        joint_state_msg.effort = [self.curr1, self.curr2, self.curr3]

        self.publisher.publish(joint_state_msg)
        self.heartbeat_publisher.publish(self.worm_heartbeat)

    def set_zero_position(self, motor):
        motor.set_zero_position()

    def on_shutdown(self):
        self.get_logger().info("Disabling Motors...")
        for motor_id, motor_controller in self.motor_controller_dict.items():
            motor_controller.disable_motor()


def main(args=None):
    rclpy.init(args=args)

    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass

    motor_controller_node.on_shutdown()
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
