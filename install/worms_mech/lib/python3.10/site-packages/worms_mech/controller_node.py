import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)

        self.state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.gait_command_subscriber = self.create_subscription(JointState, 'gait_commands', self.gait_command_callback, 10)

        self.joystick_command_subscriber = self.create_subscription(JointState, 'joystick_commands', self.joystick_command_callback, 10)

        # MORE FOR OTHER INPUT METHODS: JOYSTICK ETC.



    def joint_state_callback(self, msg):
        
        # Update the first waypoint with the current position
        self.current_pose = msg

    def joystick_command_callback(self, msg):
        self.joystick_command = msg

    def gait_command_callback(self, msg):
        
        # Update the first waypoint with the current command
        self.gait_command = msg

        pos_error = self.current_pose.position - self.gait_command.position

        # GIVEN SOME POSE ERROR CALCULATE THE NECCESARY 
        k_p = 1.0  

        # Calculate the force needed
        force = [pos_error[0] * k_p, pos_error[1] * k_p, pos_error[2] * k_p]

        # Direction of force is the sign of position error
        direction = [1 if e > 0 else -1 if e < 0 else 0 for e in pos_error]

        # Apply force direction to force vector
        force = [f * d for f, d in zip(force, direction)]

        # Publish the force as joint efforts
        joint_state_msg = JointState()
        joint_state_msg.position = self.gait_command.position
        joint_state_msg.velocity = [0.0, 0.0, 0.0] 
        joint_state_msg.effort = force

        #self.publisher.publish(joint_state_msg)
        print(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()