import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with the correct message type if different

class GoalMonitorNode(Node):
    def __init__(self):
        super().__init__('goal_monitor_node')
        self.subscription = self.create_subscription(
            String,  # Replace with the correct message type
            '/bt_navigator/transition_event',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received transition event: {msg.data}')
        if 'succeeded' in msg.data.lower():
            self.get_logger().info('Goal reached! Processing...')
            # Insert your post-arrival processing here

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
