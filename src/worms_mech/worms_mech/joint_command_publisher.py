import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time


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
        super().__init__('joint_command_publisher')

         # Construct the path to the CSV file
        spreadsheet_path = os.path.expanduser('~/worms_mech_ws/src/worms_mech/worms_mech/database.csv')

        mac_address = get_mac_address()

        worm_info = find_robot_info(mac_address, spreadsheet_path)

         # Check if worm_info is not None
        if worm_info is not None:
            self.worm_id = worm_info['Species']

        print(f"{self.worm_id} Has Been Initialized")

        self.joint_commands_topic = f'{self.worm_id}_joint_commands'
        self.joint_states_topic = f'{self.worm_id}_joint_states'

        # Initialize publisher
        self.publisher = self.create_publisher(JointState, self.joint_commands_topic, 10)

        # Define motor command sequences
        self.motor_commands = [

            {'position': [20, 0, 0], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # First set of commands
            {'position': [20, 20, 0], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # Second set
            {'position': [20, 20, 20], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]}   # Third set
            
        ]

        # Initialize command index
        self.command_index = 0

        # Timer for sending commands every 2 seconds
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
            self.publisher.publish(joint_state_msg)

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