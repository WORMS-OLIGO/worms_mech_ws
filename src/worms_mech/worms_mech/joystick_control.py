import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy


class JoystickControlNode(Node):

    def __init__(self):
        super().__init__('joystick_control')

        # Subscription to joystick
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )

        # Subscription to joint states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )


        # Publisher
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)

        # Logger to show Node is Not Working
        self.get_logger().info("Joystick Parser Initialized")

        # Current motor positions
        self.current_motor_positions = [0, 0, 0]  # Three motors
        self.commanded_motor_velocity = [0, 0, 0]  # Three motors
        self.commanded_motor_effort = [0, 0, 0]  # Three motors


        # Threshold for Axis
        self.threshold = .5

       

    def joystick_callback(self, msg):


        # Handle the incoming joystick messages here 
        # ---------------------------------------------------------------------------------------

        # 1) Read in Joystick Messages and extract axis values

        self.current_motor_positions = [0, 0, 0]  # Three motors
        self.commanded_motor_velocity = [0, 0, 0]  # Three motors
        self.commanded_motor_effort = [0, 0, 0]  # Three motors

        if abs(msg.axes[6])>self.threshold:
            if msg.axes[6]>0:
                increment = 1
                print("Positive Motion Triggered")
                self.commanded_motor_velocity = [50, 0, 0]  # Three motors
                self.commanded_motor_effort = [2, 0, 0]  # Three motors
                

            else:
                increment = -1
                print("Negative Motion Triggered")
                self.current_motor_positions[0] += increment
                self.commanded_motor_velocity = [-50, 0, 0]  # Three motors
                self.commanded_motor_effort = [-2, 0, 0]  # Three motors

        if abs(msg.axes[3])>self.threshold:
            if msg.axes[3]>0:
                increment = 1
                print("Positive Motion Triggered")
                self.commanded_motor_velocity = [0, 50, 0]  # Three motors
                self.commanded_motor_effort = [0, 2, 0]  # Three motors
                

            else:
                increment = -1
                print("Negative Motion Triggered")
                self.current_motor_positions[0] += increment
                self.commanded_motor_velocity = [0, -50, 0]  # Three motors
                self.commanded_motor_effort = [0, -2, 0]  # Three motors

        if abs(msg.axes[2])>self.threshold:
            if msg.axes[2]>0:
                increment = 1
                print("Positive Motion Triggered")
                self.commanded_motor_velocity = [0, 0, 50]  # Three motors
                self.commanded_motor_effort = [0, 0, 2]  # Three motors
                

            else:
                increment = -1
                print("Negative Motion Triggered")
                self.current_motor_positions[0] += increment
                self.commanded_motor_velocity = [0, 0, -50]  # Three motors
                self.commanded_motor_effort = [0, 0, -2]  # Three motors

               
        
        else:
            print("Not Active")
            
        # for i in range(0,2):
        #     if abs(msg.axes[i])>self.threshold:
        #         if msg.axes[i]>0:
        #             increment = 1
        #         else:
        #             increment = -1
        #         self.current_motor_positions[i+1] += increment

        # EXAMPLE BUT THE CORRECT AXIS MUST BE SELECTED AND ALSO SET THE VARIABLES TO THE CLASS VARIABLES
        #for i in range(3):
            
            #if abs(msg.axes[i]) > self.threshold:
                # increment = 1 if msg.axes[i] > 0 else -1
                # self.current_motor_positions[i] += increment




        # After you assign all three values of joint positions Call this Line and the Robot will Move
        self.publish_joint_command()


    # ---------------------------------------------------------------------------------------

    def publish_joint_command(self):

        command = {'position': [0, 0, 0], 'velocity': [0, 0, 0], 'effort': self.commanded_motor_effort}

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

    def joint_state_callback(self, msg):
        # Assuming motors correspond to the first three joints in the list
        self.current_motor_positions = msg.position[:3]

    def set_zero_position(self, motor):
        motor.set_zero_position()

    def on_shutdown(self):
        self.get_logger().info("Disabling Motors...")
        for motor_id, motor_controller in self.motor_controller_dict.items():
            motor_controller.disable_motor()


def main(args=None):
    rclpy.init(args=args)

    joystick_control_node = JoystickControlNode()

    try:
        rclpy.spin(joystick_control_node)
    except KeyboardInterrupt:
        pass

    joystick_control_node.on_shutdown()
    joystick_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
