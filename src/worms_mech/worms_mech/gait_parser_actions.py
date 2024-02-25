import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('gait_parser_actions')

        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)

        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.subscription = self.create_subscription(String, 'actions', self.actions_callback, 10)

        self.execute_timer_callback = False

        # Define waypoints (motor commands as positions here)
        # TODO: NEGATE EVERYTHING BC IT IS NOW RUNNING ON FR
        self.br_step_waypoints = [
            [0, 145, 175], 
            [30, 145, 175], #Dead Position to Prob Position
            [30, 135, 175],
              
        ]
        self.br_prone_waypoints = [
            [0, -135, 175],
            #delete later
            [1, -135, 175],
        ]

        self.br_lift_waypoints = [
            [30, -45, 35],
            #delete later
            [0, 145, 175]
        ]

        # PROPEL?????

        # Interpolate waypoints
        self.br_step_interpolated_positions = self.interpolate_waypoints(self.br_step_waypoints, .5)
        self.br_prone_interpolated_positions = self.interpolate_waypoints(self.br_prone_waypoints, .5)
        self.br_lift_interpolated_positions = self.interpolate_waypoints(self.br_lift_waypoints, .5)

        # Initialize index for interpolated positions
        self.position_index = 0

        # Timer for sending commands at each interval (adjust time as needed)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joint_state_callback(self, msg):
        
        # Update the first waypoint with the current position
        self.current_pose = msg

        if hasattr(self, 'current_pose') and self.current_pose is not None:
            position_str = ', '.join([f"{p:.2f}" for p in self.current_pose.position])
            self.get_logger().info(f'Path complete. Holding Position at: [{position_str}]')

    def interpolate_waypoints(self, waypoints, interval_degrees):
        interpolated = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            distance = np.linalg.norm(np.array(end) - np.array(start))
            steps = int(distance / interval_degrees)
            for step in range(steps + 1):
                interpolated.append(np.array(start) + (np.array(end) - np.array(start)) * step / steps)
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
            self.timer.cancel()

    def actions_callback(self, msg):
        if msg.data == "step":
            self.execute_timer_callback = True
            self.interpolated_positions = self.br_step_interpolated_positions
            self.timer_callback()
            self.action = "step"
        if msg.data == "lift":
            self.execute_timer_callback = True
            self.interpolated_positions = self.br_lift_interpolated_positions
            self.timer_callback()  
            print("heard lift")          
        if msg.data == "prone":
            self.execute_timer_callback = True
            self.interpolated_positions = self.br_prone_interpolated_positions
            self.timer_callback()
            print("heard prone")
            self.get_logger().info(f"Heard command: {self.msg.data}")
            

    

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = JointCommandPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(joint_command_publisher, executor=executor)

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
