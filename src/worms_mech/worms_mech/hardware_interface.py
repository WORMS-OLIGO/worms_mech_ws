import rclpy
from rclpy.node import Node
from motor_driver.canmotorlib import CanMotorController
from sensor_msgs.msg import JointState


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('hardware_interface')
        
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

            if(motor_id == 1):
                self.pos1, self.vel1, self.curr1 = motor_controller.send_deg_command(pos_command, vel_command, Kp, Kd, K_ff)
                
            elif(motor_id == 2):
                self.pos2, self.vel2, self.curr2 = motor_controller.send_deg_command(pos_command, vel_command, Kp, Kd, K_ff)
            
            elif(motor_id == 3):
                self.pos3, self.vel3, self.curr3 = motor_controller.send_deg_command(pos_command, vel_command, Kp, Kd, K_ff)

            else:
                print("Motor Identification Error")

        
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["motor_1", "motor_2", "motor_3"]
        joint_state_msg.position = [self.pos1, self.pos2, self.pos3]
        joint_state_msg.velocity = [self.vel1, self.vel2, self.vel3]
        joint_state_msg.effort = [self.curr1, self.curr2, self.curr3]

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
