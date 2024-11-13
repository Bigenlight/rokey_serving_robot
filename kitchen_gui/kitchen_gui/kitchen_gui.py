# kitchen_gui/kitchen_gui.py

import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGroupBox, QButtonGroup, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from std_msgs.msg import String
from custom_interface.srv import Order  # Import Order service


class KitchenGUINode(Node, QObject):
    order_received_signal = pyqtSignal(object)  # Signal to communicate with GUI thread

    def __init__(self):
        Node.__init__(self, 'kitchen_gui')
        QObject.__init__(self)
        self.alarm_buttons = {}
        self.selected_table = None

        # Remove the order topic subscription
        # self.order_subscriber = self.create_subscription(String, 'order_topic', self.handle_order_topic, 10)
        self.call_subscriber = self.create_subscription(String, 'call_topic', self.handle_call_topic, 10)

        # Create service server for Order service
        self.order_service = self.create_service(Order, 'send_order', self.handle_order_service)

        # GUI 설정
        self.app = QApplication(sys.argv)
        self.window = KitchenGUI(self)
        self.window.show()

        # Connect the signal
        self.order_received_signal.connect(self.window.display_order)

    def handle_order_service(self, request, response):
        self.get_logger().info(f"Received order from table {request.table_number}: {request.menu}")

        # Create an Event to wait for the user's response
        self.order_event = threading.Event()
        self.order_response = None

        # Emit the signal to the GUI thread
        self.order_received_signal.emit(request)

        # Wait for the user's response (with a timeout)
        if not self.order_event.wait(timeout=30):
            # Timeout
            self.order_response = '주문 수락'  # Default to accepting the order
            self.get_logger().info(f"Order from table {request.table_number} accepted by default due to timeout.")

        # Set the response
        response.response = [self.order_response]
        return response

    def handle_call_topic(self, msg):
        self.get_logger().info(f"Received call for table: {msg.data}")
        self.window.activate_alarm(msg.data)

    def run(self):
        self.app.exec_()


class KitchenGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("주방 GUI")
        self.setGeometry(100, 100, 1000, 600)
        self.alarm_buttons = {}
        self.selected_table = None

        self.table_number_to_name = {
            1: '테이블A',
            2: '테이블B',
            3: '테이블C'
        }

        self.order_details_labels = {}
        self.order_reject_buttons = {}
        self.initUI()
        self.current_order_request = None  # Store the current order request

    def initUI(self):
        main_layout = QHBoxLayout(self)

        # 왼쪽 칼럼: 주문 내역과 주문 내역 표시 영역
        left_column = QVBoxLayout()
        order_label = QLabel("주문 내역")
        order_label.setAlignment(Qt.AlignCenter)
        order_label.setFixedHeight(30)
        left_column.addWidget(order_label)

        for table in ["테이블A", "테이블B", "테이블C"]:
            table_layout = QVBoxLayout()
            table_name = QLabel(table)
            table_name.setAlignment(Qt.AlignCenter)
            table_name.setFixedHeight(20)
            table_layout.addWidget(table_name)

            order_details = QLabel("주문 내역 표시 영역")
            order_details.setStyleSheet("background-color: #003366; color: white;")
            order_details.setFixedHeight(150)
            table_layout.addWidget(order_details)

            left_column.addLayout(table_layout)

            # Store the order_details label
            self.order_details_labels[table] = order_details

        main_layout.addLayout(left_column, 1)

        # 중앙 칼럼: DB 버튼과 테이블별 버튼들
        center_column = QVBoxLayout()
        db_button = QPushButton("DB 확인")
        db_button.setFixedHeight(30)
        center_column.addWidget(db_button)

        for table in ["테이블A", "테이블B", "테이블C"]:
            table_group = QGroupBox(table + " 버튼")
            table_layout = QVBoxLayout()

            order_reject_button = QPushButton("주문 거절")
            order_reject_button.setEnabled(False)
            order_reject_button.setFixedHeight(30)
            # No need to connect the clicked signal here; we'll connect it when we display the order
            table_layout.addWidget(order_reject_button)

            # Store the order_reject_button
            self.order_reject_buttons[table] = order_reject_button

            out_of_stock_button = QPushButton("재료 부족")
            out_of_stock_button.setEnabled(False)
            out_of_stock_button.setFixedHeight(30)
            out_of_stock_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "재료 부족"))
            table_layout.addWidget(out_of_stock_button)

            not_in_mood_button = QPushButton("하기 싫음")
            not_in_mood_button.setEnabled(False)
            not_in_mood_button.setFixedHeight(30)
            not_in_mood_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "하기 싫음"))
            table_layout.addWidget(not_in_mood_button)

            cooking_done_button = QPushButton("조리 완료")
            cooking_done_button.setFixedHeight(30)
            cooking_done_button.clicked.connect(lambda _, tb=table: self.mark_cooking_done(tb))
            table_layout.addWidget(cooking_done_button)

            alarm_off_button = QPushButton("알람 끄기")
            alarm_off_button.setFixedHeight(30)
            alarm_off_button.setStyleSheet("background-color: grey;")
            alarm_off_button.setEnabled(False)
            alarm_off_button.clicked.connect(lambda _, ab=alarm_off_button: self.disable_alarm_button(ab))
            table_layout.addWidget(alarm_off_button)

            self.alarm_buttons[table] = alarm_off_button
            table_group.setLayout(table_layout)
            center_column.addWidget(table_group)

        main_layout.addLayout(center_column, 1)

        # 오른쪽 칼럼: 수동 조종 영역
        right_column = QVBoxLayout()
        control_label = QLabel("수동 조종")
        control_label.setAlignment(Qt.AlignCenter)
        control_label.setFixedHeight(30)
        right_column.addWidget(control_label)

        table_selection = QGroupBox("테이블 선택")
        table_selection_layout = QVBoxLayout()

        self.table_button_group = QButtonGroup()
        self.table_button_group.setExclusive(False)

        for table in ["테이블 A", "테이블 B", "테이블 C"]:
            table_button = QPushButton(table)
            table_button.setCheckable(True)
            table_button.clicked.connect(lambda _, tb=table: self.select_table(tb))
            self.table_button_group.addButton(table_button)
            table_selection_layout.addWidget(table_button)

        table_selection.setLayout(table_selection_layout)
        right_column.addWidget(table_selection)

        function_group = QGroupBox("기능")
        function_layout = QHBoxLayout()
        for function in ["긴급 정지", "주방 복귀", "로봇 보내기"]:
            function_button = QPushButton(function)
            function_button.setFixedHeight(30)
            function_button.clicked.connect(lambda _, fn=function: self.perform_function(fn))
            function_layout.addWidget(function_button)
        function_group.setLayout(function_layout)
        right_column.addWidget(function_group)

        main_layout.addLayout(right_column, 1)

        self.setLayout(main_layout)

    def display_order(self, request):
        # Display the order details in the GUI
        self.current_order_request = request
        # Get the table name
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")
        # Update the order display area
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            order_text = '\n'.join(request.menu)
            order_details_label.setText(order_text)
        else:
            self.node.get_logger().error(f"No order details label found for table {table_name}")

        # Activate the '주문 거절' button for this table
        order_reject_button = self.order_reject_buttons.get(table_name)
        if order_reject_button:
            order_reject_button.setEnabled(True)
            # Disconnect previous connections
            try:
                order_reject_button.clicked.disconnect()
            except TypeError:
                pass
            order_reject_button.clicked.connect(self.reject_order)
        else:
            self.node.get_logger().error(f"No order reject button found for table {table_name}")

    def activate_alarm(self, table):
        button = self.alarm_buttons.get(table)
        if button:
            button.setStyleSheet("background-color: red;")
            button.setEnabled(True)

    def disable_alarm_button(self, button):
        button.setStyleSheet("background-color: grey;")
        button.setEnabled(False)

    def select_table(self, table):
        self.selected_table = table if self.selected_table != table else None
        for button in self.table_button_group.buttons():
            button.setStyleSheet("background-color: yellow;" if button.text() == self.selected_table else "")

    def perform_function(self, function):
        if self.selected_table:
            self.node.get_logger().info(f"{self.selected_table}에서 {function} 기능을 수행합니다.")
        else:
            self.node.get_logger().info("선택된 테이블이 없습니다.")

    def reject_order(self):
        # User clicked '주문 거절' button
        self.node.order_response = '주문 거절'
        self.node.order_event.set()
        # Disable the button
        table_name = self.table_number_to_name.get(self.current_order_request.table_number)
        order_reject_button = self.order_reject_buttons.get(table_name)
        if order_reject_button:
            order_reject_button.setEnabled(False)
        # Optionally, clear the order details
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            order_details_label.setText("주문 거절됨")

    def send_reject_reason(self, table, reason):
        self.node.get_logger().info(f"{table}의 주문 거절 이유: {reason}")

    def mark_cooking_done(self, table):
        self.node.get_logger().info(f"{table}의 조리가 완료되었습니다.")

def main(args=None):
    rclpy.init(args=args)
    kitchen_gui_node = KitchenGUINode()
    try:
        kitchen_gui_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        kitchen_gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
