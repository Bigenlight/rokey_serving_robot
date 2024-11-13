import rclpy
from rclpy.node import Node
from custom_interface.srv import Order

class OrderClient(Node):
    def __init__(self):
        super().__init__('order_client')
        self.cli = self.create_client(Order, 'send_order')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the order service to be available...')
        self.request = Order.Request()

    def send_order_request(self, table_number, time, menu):
        self.request.table_number = table_number
        self.request.time = time
        self.request.menu = menu
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = OrderClient()
    
    table_number = 1
    time = ['12:00']
    menu = ['Pizza', 'Soda']
    
    response = client.send_order_request(table_number, time, menu)
    client.get_logger().info(f'Response: {response.response}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
