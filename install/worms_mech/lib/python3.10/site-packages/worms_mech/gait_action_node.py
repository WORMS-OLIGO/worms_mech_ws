import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):

    def __init__(self):
        super().__init__('gait_action_node')
        self.publisher_ = self.create_publisher(String, 'actions', 10)
        
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
            print("MOTION COMPLETED")
            self.publish_command()  # Publish next command if available
        elif(self.current_index == 0):
            self.publish_command()
        else:
            print(str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()

    # Get the list of commands from the user
    command_publisher.command_list = input("Enter a list of commands (separated by spaces): ").split()

    if command_publisher.command_list == ["run_stand_forward_gait"]:
        command_publisher.command_list = ['stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
    elif command_publisher.command_list == ["run_field_forward_gait"]:
        command_publisher.command_list = ['field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
    elif command_publisher.command_list == ["run_stand_reverse_gait"]:
        command_publisher.command_list = ['reverse', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
    elif command_publisher.command_list == ["run_field_forward_gait"]:
        command_publisher.command_list = ['reverse', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
    elif command_publisher.command_list == ["run_full_stand_gait"]:
        command_publisher.command_list = ['stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'reverse', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone', 'stand_step', 'stand_stand','stand_propel', 'stand_prone']
    elif command_publisher.command_list == ["run_full_field_gait"]:
        command_publisher.command_list = ['field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'reverse', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone', 'field_step', 'field_stand','field_propel', 'field_prone']
    else:
        command_publisher.command_list = command_publisher.command_list


    try:
        rclpy.spin(command_publisher)
    except KeyboardInterrupt:
        pass
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
