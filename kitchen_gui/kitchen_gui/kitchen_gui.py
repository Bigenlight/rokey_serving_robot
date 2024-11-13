# kitchen_gui/kitchen_gui.py

import sys
import threading
import queue
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGroupBox, QButtonGroup, QMessageBox
)
from PyQt5.QtCore import Qt
from custom_interface.srv import Order  # Import Order service

class KitchenGUINode(Node):
    def __init__(self, order_queue):
        super().__init__('kitchen_gui')
        self.order_queue = order_queue
        self.alarm_buttons = {}
        self.selected_table = None

        # Create service server for Order service
        self.order_service = self.create_service(Order, 'send_order', self.handle_order_service)

        # Initialize table number to name mapping
        self.table_number_to_name = {
            1: '테이블A',
            2: '테이블B',
            3: '테이블C'
        }

    def handle_order_service(self, request, response):
        self.get_logger().info(f"Received order from table {request.table_number}: {request.menu}")

        # Put the order request and response in the queue for the GUI to process
        self.order_queue.put((request, response))

        # The GUI thread will handle setting the response after user interaction
        return response

def ros_spin(node):
    rclpy.spin(node)

class KitchenGUI(QWidget):
    def __init__(self, order_queue, node):
        super().__init__()
        self.order_queue = order_queue
        self.node = node
        self.setWindowTitle("주방 GUI")
        self.setGeometry(100, 100, 1000, 600)
        self.alarm_buttons = {}
        self.selected_table = None
        self.current_order_request = None  # Store the current order request
        self.current_order_response = None  # Store the current order response

        # Initialize order_details_labels and order_reject_buttons
        self.order_details_labels = {}
        self.order_accept_buttons = {}
        self.out_of_stock_buttons = {}
        self.not_in_mood_buttons = {}

        # Mapping table numbers to names
        self.table_number_to_name = self.node.table_number_to_name

        self.initUI()

        # Start a timer to periodically check the order queue
        self.timer = self.startTimer(100)  # Check every 100 ms

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
            order_details.setAlignment(Qt.AlignTop | Qt.AlignLeft)
            order_details.setWordWrap(True)
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

            # Renamed to order_accept_button for clarity
            order_accept_button = QPushButton("주문 수락")
            order_accept_button.setEnabled(False)
            order_accept_button.setFixedHeight(30)
            # We'll connect this when an order is received
            table_layout.addWidget(order_accept_button)

            # Store the order_accept_button
            self.order_accept_buttons[table] = order_accept_button

            # Out of Stock Button
            out_of_stock_button = QPushButton("재료 부족")
            out_of_stock_button.setEnabled(False)
            out_of_stock_button.setFixedHeight(30)
            out_of_stock_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "재료 부족"))
            table_layout.addWidget(out_of_stock_button)

            # Store the out_of_stock_button
            self.out_of_stock_buttons[table] = out_of_stock_button

            # Not in Mood Button
            not_in_mood_button = QPushButton("하기 싫음")
            not_in_mood_button.setEnabled(False)
            not_in_mood_button.setFixedHeight(30)
            not_in_mood_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "하기 싫음"))
            table_layout.addWidget(not_in_mood_button)

            # Store the not_in_mood_button
            self.not_in_mood_buttons[table] = not_in_mood_button

            # Cooking Done Button
            cooking_done_button = QPushButton("조리 완료")
            cooking_done_button.setFixedHeight(30)
            cooking_done_button.clicked.connect(lambda _, tb=table: self.mark_cooking_done(tb))
            table_layout.addWidget(cooking_done_button)

            # Alarm Off Button
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

    def timerEvent(self, event):
        while not self.order_queue.empty():
            request, response = self.order_queue.get()
            self.display_order(request, response)

    def display_order(self, request, response):
        # Display the order details in the GUI
        self.current_order_request = request
        self.current_order_response = response

        # Get the table name
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")

        # Update the order display area
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            order_text = '\n'.join(request.menu)
            order_details_label.setText(order_text)
        else:
            self.show_error(f"No order details label found for table {table_name}")

        # Activate the '주문 수락', '재료 부족', and '하기 싫음' buttons for this table
        order_accept_button = self.order_accept_buttons.get(table_name)
        out_of_stock_button = self.out_of_stock_buttons.get(table_name)
        not_in_mood_button = self.not_in_mood_buttons.get(table_name)

        if order_accept_button and out_of_stock_button and not_in_mood_button:
            order_accept_button.setEnabled(True)
            out_of_stock_button.setEnabled(True)
            not_in_mood_button.setEnabled(True)

            # Disconnect previous connections to avoid multiple slots being connected
            try:
                order_accept_button.clicked.disconnect()
            except TypeError:
                pass
            try:
                out_of_stock_button.clicked.disconnect()
            except TypeError:
                pass
            try:
                not_in_mood_button.clicked.disconnect()
            except TypeError:
                pass

            # Connect buttons to their respective handlers
            order_accept_button.clicked.connect(lambda: self.accept_order(table_name))
            # The out_of_stock and not_in_mood buttons are already connected to send_reject_reason
            # which will be modified to disable buttons after being clicked
        else:
            self.show_error(f"One or more buttons not found for table {table_name}")

    def accept_order(self, table_name):
        if self.current_order_request and self.current_order_response:
            self.node.get_logger().info(f"{table_name}의 주문을 수락합니다.")
            self.current_order_response.response = ['주문 수락']
            self.current_order_response = None  # Reset the response
            self.current_order_request = None  # Reset the request

            # Disable all three buttons
            self.disable_action_buttons(table_name)

            # Optionally, clear the order details
            order_details_label = self.order_details_labels.get(table_name)
            if order_details_label:
                order_details_label.setText("주문 수락됨")
                QMessageBox.information(self, "주문 수락", f"{table_name}의 주문이 수락되었습니다.")
        else:
            self.show_error("현재 처리 중인 주문이 없습니다.")

    def send_reject_reason(self, table, reason):
        if self.current_order_request and self.current_order_response:
            self.node.get_logger().info(f"{table}의 주문 거절 이유: {reason}")
            self.current_order_response.response = [reason]
            self.current_order_response = None  # Reset the response
            self.current_order_request = None  # Reset the request

            # Disable all three buttons
            self.disable_action_buttons(table)

            # Optionally, update the order details
            order_details_label = self.order_details_labels.get(table)
            if order_details_label:
                order_details_label.setText(f"주문 거절됨: {reason}")
                QMessageBox.information(self, "주문 거절", f"{table}의 주문이 거절되었습니다.\n이유: {reason}")
        else:
            self.show_error("현재 처리 중인 주문이 없습니다.")

    def disable_action_buttons(self, table_name):
        # Disable '주문 수락', '재료 부족', and '하기 싫음' buttons
        order_accept_button = self.order_accept_buttons.get(table_name)
        out_of_stock_button = self.out_of_stock_buttons.get(table_name)
        not_in_mood_button = self.not_in_mood_buttons.get(table_name)

        if order_accept_button:
            order_accept_button.setEnabled(False)
        if out_of_stock_button:
            out_of_stock_button.setEnabled(False)
        if not_in_mood_button:
            not_in_mood_button.setEnabled(False)

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
            QMessageBox.information(self, "기능 수행", f"{self.selected_table}에서 {function} 기능을 수행합니다.")
        else:
            self.node.get_logger().info("선택된 테이블이 없습니다.")
            QMessageBox.warning(self, "경고", "선택된 테이블이 없습니다.")

    def mark_cooking_done(self, table):
        self.node.get_logger().info(f"{table}의 조리가 완료되었습니다.")
        QMessageBox.information(self, "조리 완료", f"{table}의 조리가 완료되었습니다.")

    def show_error(self, message):
        QMessageBox.critical(self, "오류", message)

def main(args=None):
    rclpy.init(args=args)

    order_queue = queue.Queue()
    node = KitchenGUINode(order_queue)

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = KitchenGUI(order_queue, node)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
