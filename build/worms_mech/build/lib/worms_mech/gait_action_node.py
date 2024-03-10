import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('gait_action_node')
        self.publisher_ = self.create_publisher(String, 'actions', 10)
        self.timer_ = self.create_timer(0.5, self.publish_command)
        self.get_logger().info('Command Publisher node initialized')

        self.state_subscriber = self.create_subscription(String, "/coordination", self.action_callback, 10)
        self.command_list = []
        self.current_index = 0

    def publish_command(self):
        if self.current_index < len(self.command_list):
            user_input = self.command_list[self.current_index]
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            self.current_index += 1

    def action_callback(self, msg):
        if msg.data == "done":
            self.publish_command()  # Publish next command if available

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()

    # Get the list of commands from the user
    command_publisher.command_list = input("Enter a list of commands (separated by spaces): ").split()

    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
