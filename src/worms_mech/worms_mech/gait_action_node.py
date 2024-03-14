import rclpy
from rclpy.node import Node
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

def find_robot_name(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        return None

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('gait_action_node')
        self.publisher_ = self.create_publisher(String, 'actions', 10)
        self.timer = self.create_timer(0.5, self.gait_manager)
        
        self.get_logger().info('Command Publisher node initialized')

        # Construct the path to the CSV file for worm info
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        worm_id = find_robot_name(mac_address, spreadsheet_path)

        duck_coordination_topic = 'Duck_coordination'
        swan_coordination_topic = 'Swan_coordination'
        lion_coordination_topic = 'Lion_coordination'
        pony_coordination_topic = 'Pony_coordination'
        goat_coordination_topic = 'Goat_coordination'
        frog_coordination_topic = 'Frog_coordination'


        self.duck_state_subscriber = self.create_subscription(String, duck_coordination_topic, self.duck_action_callback, 10)
        self.swan_state_subscriber = self.create_subscription(String, swan_coordination_topic, self.swan_action_callback, 10)
        self.lion_state_subscriber = self.create_subscription(String, lion_coordination_topic, self.lion_action_callback, 10)
        self.pony_state_subscriber = self.create_subscription(String, pony_coordination_topic, self.pony_action_callback, 10)
        self.goat_state_subscriber = self.create_subscription(String, goat_coordination_topic, self.goat_action_callback, 10)
        self.frog_state_subscriber = self.create_subscription(String, frog_coordination_topic, self.frog_action_callback, 10)

        #1 is ready to go

        self.goat_status = 1
        
        self.duck_status = 1

        self.frog_status = 1

        self.pony_status = 1

        self.swan_status = 1

        self.lion_status = 1


        self.command_list = []
        self.current_index = 0

    def publish_command(self):
        if self.current_index < len(self.command_list):
            user_input = self.command_list[self.current_index]
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            self.current_index += 1

    def duck_action_callback(self, msg):
        if msg.data == "done":
            self.duck_status = 1
        else:
            self.duck_status = 0

    def swan_action_callback(self, msg):
        if msg.data == "done":
            self.swan_status = 1
        else:
            self.swan_status = 0

    def lion_action_callback(self, msg):
        if msg.data == "done":
            self.lion_status = 1
        else:
            self.lion_status = 0

    def pony_action_callback(self, msg):
        if msg.data == "done":
            self.pony_status = 1
        else:
            self.pony_status = 0

    def goat_action_callback(self, msg):
        if msg.data == "done":
            self.goat_status = 1
        else:
            self.goat_status = 0

    def frog_action_callback(self, msg):
        if msg.data == "done":
            self.frog_status = 1
        else:
            self.frog_status = 0


    def gait_manager(self):

        self.command_publisher = CommandPublisher()

        # Get the list of commands from the user

        if(self.goat_status and self.pony_status and self.duck_status and self.swan_status):
            self.command_publisher.command_list.append(input("Enter a list of commands (separated by spaces): ").split())
            self.publish_command()
            # if self.command_publisher.command_list == ["run_stand_forward_gait"]:
            #     self.command_publisher.command_list = ['stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
            # elif self.command_publisher.command_list == ["run_field_forward_gait"]:
            #     self.command_publisher.command_list = ['field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
            # elif self.command_publisher.command_list == ["run_stand_reverse_gait"]:
            #     self.command_publisher.command_list = ['reverse', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
            # elif self.command_publisher.command_list == ["run_field_forward_gait"]:
            #     self.command_publisher.command_list = ['reverse', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
            # elif self.command_publisher.command_list == ["run_full_stand_gait"]:
            #     self.command_publisher.command_list = ['stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'reverse', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
            # elif self.command_publisher.command_list == ["run_full_field_gait"]:
            #     self.command_publisher.command_list = ['field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'reverse', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
            # else:
            #     self.command_publisher.command_list = self.command_publisher.command_list
        elif(self.current_index == 0):
            self.publish_command()
        else:
            print("WAITING FOR ALL WORMS TO COMPLETE ACTION")
    



def main(self,args=None):
    rclpy.init(args=args)
    
    try:
        rclpy.spin(self.command_publisher)
    except KeyboardInterrupt:
        pass
    self.command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
