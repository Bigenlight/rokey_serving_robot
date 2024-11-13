# user_gui/service_server.py

import rclpy
from rclpy.node import Node
from user_gui.srv import SendOrder, CallStaff

class KitchenServiceServer(Node):
    def __init__(self):
        super().__init__('kitchen_service_server')
        self.send_order_service = self.create_service(SendOrder, 'send_order', self.send_order_callback)
        self.call_staff_service = self.create_service(CallStaff, 'call_staff', self.call_staff_callback)
        self.get_logger().info('Kitchen Service Server is ready.')

    def send_order_callback(self, request, response):
        self.get_logger().info(f'Received order: {request.items} with quantities: {request.quantities}')
        # 주문 처리 로직 추가
        response.success = True
        response.message = 'Order received successfully.'
        return response

    def call_staff_callback(self, request, response):
        self.get_logger().info(f'Staff called by: {request.requestor_id}')
        # 직원 호출 로직 추가
        response.success = True
        response.message = 'Staff has been called.'
        return response

def main(args=None):
    rclpy.init(args=args)
    kitchen_service_server = KitchenServiceServer()
    rclpy.spin(kitchen_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
