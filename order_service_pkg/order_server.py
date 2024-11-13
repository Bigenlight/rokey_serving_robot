import rclpy
from rclpy.node import Node
from custom_interface.srv import Order

class OrderServer(Node):
    def __init__(self):
        super().__init__('order_server')
        self.srv = self.create_service(Order, 'send_order', self.handle_order_request)
        self.get_logger().info('Order Server Ready to Receive Orders')

    def handle_order_request(self, request, response):
        self.get_logger().info(f'Received order for table {request.table_number}')
        response.response = [f"Order received at table {request.table_number} at {request.time} with menu {request.menu}"]
        self.get_logger().info(f'Order received at table {request.table_number} at {request.time} with menu {request.menu}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = OrderServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
