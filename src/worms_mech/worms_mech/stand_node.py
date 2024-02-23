import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'stand', 10)
        self.timer_ = self.create_timer(0.5, self.publish_command)
        self.get_logger().info('Command Publisher node initialized')

    def publish_command(self):
        user_input = input("Enter command: ")
        if user_input == "stand":
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()
    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
