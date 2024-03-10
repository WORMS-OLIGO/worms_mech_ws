import rclpy
from rclpy.node import Node
from motor_driver.canmotorlib import CanMotorController
from sensor_msgs.msg import JointState


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('testbed_interface')
        
        self.motor_controller_dict = {}
        
        # Hardcoded to match pican board
        can_device = 'can0'

        self.motor_controller = CanMotorController(can_device, 3, motor_type="AK80_6_V2")

        print("Creating Subscriber")
        self.subscription = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_commands_callback,
            10)
        
        print("Subscriber Made")

        self.pos = 0

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info("Enabling Motors...")
        
        self.motor_controller.enable_motor()

    def joint_commands_callback(self, msg):
        # Handle the incoming joint state command messages here
    
        # Assuming Kp and Kd as constants for this example
        Kp = 15
        Kd = 2

        pos_command = self.pos
        vel_command = msg.velocity[0]
        K_ff = msg.effort[0]
        print(K_ff)

        self.pos, self.vel, self.curr = self.motor_controller.send_deg_command(pos_command, vel_command, Kp, Kd, K_ff)
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name.append("motor_1")
        joint_state_msg.position.append(self.pos)
        joint_state_msg.velocity.append(self.vel)
        joint_state_msg.effort.append(self.curr)  # Using effort to represent torque/acc. Adjust as needed.

        self.publisher.publish(joint_state_msg)

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
