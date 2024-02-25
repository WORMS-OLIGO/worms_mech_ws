# MotorControllerNode

import rclpy
from rclpy.node import Node
from motor_driver.canmotorlib import CanMotorController
from sensor_msgs.msg import JointState

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('hardware_interfacev2')
        
        self.motor_controller_dict = {}
        
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
            'joint_commands',
            self.joint_commands_callback,
            10)
        
        print("Subscriber Made")

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info("Enabling Motors...")
        for motor_id, motor_controller in self.motor_controller_dict.items():
            motor_controller.enable_motor()

    def joint_commands_callback(self, msg):
        # Forward the received joint state command messages to the hardware

        self.publisher.publish(msg)

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
