import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Initialize publisher
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)

        # Define motor command sequences
        self.motor_commands = [

            {'position': [20, 20, 20], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # First set of commands
            {'position': [-20, -20, -20], 'velocity': [0, 0, 0], 'effort': [0, 0, 0]},  # Second set
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