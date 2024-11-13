#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_services.srv import SendOrder, CallStaff
from std_msgs.msg import String
import json
import time  # sleep 사용 시 필요

class KitchenService(Node):
    def __init__(self):
        super().__init__('kitchen_service')
        self.send_order_service = self.create_service(SendOrder, 'send_order', self.send_order_callback)
        self.call_staff_service = self.create_service(CallStaff, 'call_staff', self.call_staff_callback)
        
        # 주문 상태 퍼블리셔
        self.order_status_publisher = self.create_publisher(String, 'order_status', 10)
        # 직원 호출 상태 퍼블리셔
        self.staff_call_publisher = self.create_publisher(String, 'staff_call_status', 10)
        
        self.get_logger().info('Kitchen Service 서버가 시작되었습니다.')

    def send_order_callback(self, request, response):
        self.get_logger().info(f"주문 수신: {request.items} / 수량: {request.quantities}")
        
        # 주문 처리 로직 추가 (여기서는 시뮬레이션)
        order = {
            'items': request.items,
            'quantities': request.quantities,
            'status': '처리 중'
        }
        self.publish_order_status(order)
        
        # 주문 처리 시뮬레이션 (예: 5초 후 완료)
        time.sleep(5)
        order['status'] = '완료'
        self.publish_order_status(order)
        
        response.success = True
        response.message = "주문이 성공적으로 접수되었습니다."
        return response

    def call_staff_callback(self, request, response):
        self.get_logger().info(f"직원 호출 요청: {request.requestor_id}")
        
        # 직원 호출 처리 로직 추가 (여기서는 시뮬레이션)
        call = {
            'requestor_id': request.requestor_id,
            'status': '호출 중'
        }
        self.publish_staff_call_status(call)
        
        # 직원 도착 시뮬레이션 (예: 3초 후 완료)
        time.sleep(3)
        call['status'] = '도착'
        self.publish_staff_call_status(call)
        
        response.success = True
        response.message = f"{request.requestor_id}에서 직원 호출이 요청되었습니다."
        return response

    def publish_order_status(self, order):
        message = String()
        message.data = json.dumps(order)
        self.order_status_publisher.publish(message)
        self.get_logger().info(f"주문 상태 업데이트: {message.data}")

    def publish_staff_call_status(self, call):
        message = String()
        message.data = json.dumps(call)
        self.staff_call_publisher.publish(message)
        self.get_logger().info(f"직원 호출 상태 업데이트: {message.data}")

def main(args=None):
    rclpy.init(args=args)
    kitchen_service = KitchenService()
    rclpy.spin(kitchen_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
