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

        self.command_subscriber = self.create_subscription(JointState, 'gait_commands', self.gait_command_callback, 10)

        # MORE FOR OTHER INPUT METHODS: JOYSTICK ETC.



    def joint_state_callback(self, msg):
        
        # Update the first waypoint with the current position
        self.current_pose = msg

    def gait_command_callback(self, msg):
        
        # Update the first waypoint with the current command
        self.gait_command = msg

        pos_error = self.current_pose.position - self.gait_command.position

        # GIVEN SOME POSE ERROR CALCULATE THE NECCESARY 
        # FORCE NEEDED AND DIRECTION OF FORCE TO CORRECT POSITION ####################################################

        pos_error = self.current_pose.position - self.gait_command.position









        force = [ff1, ff2, ff3]

        ###############################################################################################################
 
        joint_state_msg = JointState()
        joint_state_msg.position = self.gait_command.position

        joint_state_msg.velocity = [0.0, 0.0, 0.0]  # Ensuring these are also floats
        joint_state_msg.effort = force
        

        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()