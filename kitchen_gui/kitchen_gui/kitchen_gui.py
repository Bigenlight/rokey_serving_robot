# kitchen_gui/kitchen_gui.py

import sys
import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGroupBox, QMessageBox, QGridLayout, QButtonGroup, QSizePolicy
)
from PyQt5.QtCore import Qt
from custom_interface.srv import Order  # Import Order service


class KitchenGUINode(Node):
    def __init__(self, order_queue):
        super().__init__('kitchen_gui')
        self.order_queue = order_queue

        # Create service server for Order service
        self.order_service = self.create_service(Order, 'send_order', self.handle_order_service)

        # Initialize table number to name mapping (테이블1부터 테이블9까지)
        self.table_number_to_name = {i: f'테이블{i}' for i in range(1, 10)}

        # Create a publisher to send order rejection messages to user_gui.py
        self.rejection_publisher = self.create_publisher(String, 'order_rejections', 10)

        # Dictionary to keep track of pending orders: request_id -> (Event, response)
        self.pending_orders = {}

        # Mutex to protect access to pending_orders
        self.pending_orders_mutex = threading.Lock()

        # Create a subscriber to listen for staff call messages
        self.staff_call_subscription = self.create_subscription(
            String,
            'call_staff',
            self.staff_call_callback,
            10
        )

        # Create a queue for staff call messages
        self.staff_call_queue = queue.Queue()

    def handle_order_service(self, request, response):
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")
        self.get_logger().info(f"Received order from {table_name}: {request.menu}")

        # Create an Event for synchronization
        order_event = threading.Event()

        # Generate a unique request ID
        request_id = id(request)  # Using the id of the request object as an identifier

        # Store the Event and response in pending_orders
        with self.pending_orders_mutex:
            self.pending_orders[request_id] = (order_event, response)

        # Put the order request and request_id in the queue for the GUI to process
        self.order_queue.put((request, request_id))

        # Wait for the Event to be set by the GUI thread
        # Set a timeout (e.g., 60 seconds)
        if order_event.wait(timeout=60):
            # Event was set, GUI has processed the order and set the response
            with self.pending_orders_mutex:
                # Get the updated response
                _, response = self.pending_orders.pop(request_id)
            self.get_logger().info(f"Order from {table_name} processed with response: {response.response}")
            return response
        else:
            # Timeout occurred
            self.get_logger().warning(f"Order from {table_name} timed out")
            with self.pending_orders_mutex:
                # Remove the pending order
                self.pending_orders.pop(request_id, None)
            # Set a default response indicating timeout
            response.response = ['주문 처리 시간 초과']
            return response

    def update_order_response(self, request_id, response):
        with self.pending_orders_mutex:
            if request_id in self.pending_orders:
                order_event, response_obj = self.pending_orders[request_id]
                response_obj.response = response.response
                # Set the Event to unblock the service callback
                order_event.set()
            else:
                self.get_logger().error(f"Order with request_id {request_id} not found in pending orders")

    def publish_rejection(self, table, message):
        msg = String()
        msg.data = f"{table}: {message}"
        self.rejection_publisher.publish(msg)
        self.get_logger().info(f"Published rejection message: {msg.data}")

    def staff_call_callback(self, msg):
        message = msg.data
        self.get_logger().info(f"Received staff call message: {message}")
        # Put the message into the staff_call_queue
        self.staff_call_queue.put(message)


def ros_spin(node):
    rclpy.spin(node)


class DBWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("DB 확인")
        self.setGeometry(200, 200, 400, 300)  # 새 창의 크기 설정

        # DB 정보를 표시할 레이블 (예시)
        self.db_label = QLabel("DB 정보가 여기에 표시됩니다.", self)
        self.db_label.setAlignment(Qt.AlignCenter)
        self.db_label.setStyleSheet("font-size: 16px;")

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.db_label)

        self.setLayout(layout)


class KitchenGUI(QWidget):
    def __init__(self, order_queue, node):
        super().__init__()
        self.order_queue = order_queue
        self.node = node
        self.setWindowTitle("주방 GUI")
        self.setGeometry(100, 100, 1600, 900)  # 창 크기 조정

        self.current_orders = {}  # Store current orders per table

        # Initialize order_details_labels and separate button dictionaries
        self.order_details_labels = {}
        self.order_accept_buttons = {}
        self.out_of_stock_buttons = {}
        self.not_in_mood_buttons = {}
        self.cooking_done_buttons = {}

        # Mapping table numbers to names
        self.table_number_to_name = self.node.table_number_to_name

        # Get the staff call queue from the node
        self.staff_call_queue = self.node.staff_call_queue

        # Initialize DB Window as None
        self.db_window = None

        self.initUI()

        # Start a timer to periodically check the order queue and staff call queue
        self.timer = self.startTimer(100)  # Check every 100 ms

    def initUI(self):
        main_layout = QHBoxLayout(self)

        # Left+Center Column: 3x3 Grid for Tables and their Functions
        grid_layout = QGridLayout()

        tables = [f"테이블{i}" for i in range(1, 10)]  # 테이블1부터 테이블9까지

        for i, table in enumerate(tables):
            row = i // 3
            col = i % 3

            # Create a group box for each table
            group_box = QGroupBox(table)
            group_layout = QVBoxLayout()

            # Order Details Label
            order_details = QLabel("주문 내역 표시 영역")
            order_details.setStyleSheet("background-color: #003366; color: white;")
            order_details.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            order_details.setAlignment(Qt.AlignTop | Qt.AlignLeft)
            order_details.setWordWrap(True)
            group_layout.addWidget(order_details)

            # Control Buttons Layout
            buttons_layout = QHBoxLayout()

            # 주문 수락 버튼
            order_accept_button = QPushButton("주문 수락")
            order_accept_button.setEnabled(False)
            order_accept_button.setFixedHeight(30)
            order_accept_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(order_accept_button)
            self.order_accept_buttons[table] = order_accept_button

            # 재료 부족 버튼
            out_of_stock_button = QPushButton("재료 부족")
            out_of_stock_button.setEnabled(False)
            out_of_stock_button.setFixedHeight(30)
            out_of_stock_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(out_of_stock_button)
            self.out_of_stock_buttons[table] = out_of_stock_button

            # 하기 싫음 버튼
            not_in_mood_button = QPushButton("하기 싫음")
            not_in_mood_button.setEnabled(False)
            not_in_mood_button.setFixedHeight(30)
            not_in_mood_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(not_in_mood_button)
            self.not_in_mood_buttons[table] = not_in_mood_button

            # 조리 완료 버튼
            cooking_done_button = QPushButton("조리 완료")
            cooking_done_button.setEnabled(False)
            cooking_done_button.setFixedHeight(30)
            cooking_done_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(cooking_done_button)
            self.cooking_done_buttons[table] = cooking_done_button

            # Connect buttons to their respective handlers
            order_accept_button.clicked.connect(lambda _, tb=table: self.accept_order(tb))
            out_of_stock_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "재료 부족"))
            not_in_mood_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "하기 싫음"))
            cooking_done_button.clicked.connect(lambda _, tb=table: self.mark_cooking_done(tb))

            group_layout.addLayout(buttons_layout)

            group_box.setLayout(group_layout)
            group_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            grid_layout.addWidget(group_box, row, col)

            # Store the order_details label
            self.order_details_labels[table] = order_details

        main_layout.addLayout(grid_layout, 5)  # 비율 조정 (더 넓은 공간 할당)

        # Right Column: Manual Control (Optional)
        right_column = QVBoxLayout()
        control_label = QLabel("수동 조종")
        control_label.setAlignment(Qt.AlignCenter)
        control_label.setFixedHeight(30)
        control_label.setStyleSheet("font-weight: bold; font-size: 16px;")
        right_column.addWidget(control_label)

        table_selection = QGroupBox("테이블 선택")
        table_selection_layout = QVBoxLayout()

        self.selected_table = None
        self.table_button_group = QButtonGroup()
        self.table_button_group.setExclusive(True)

        for table in tables:
            table_button = QPushButton(table)
            table_button.setCheckable(True)
            table_button.clicked.connect(lambda _, tb=table: self.select_table(tb))
            self.table_button_group.addButton(table_button)
            table_selection_layout.addWidget(table_button)

        table_selection.setLayout(table_selection_layout)
        right_column.addWidget(table_selection)

        function_group = QGroupBox("기능")
        function_layout = QVBoxLayout()
        for function in ["긴급 정지", "주방 복귀", "로봇 보내기"]:
            function_button = QPushButton(function)
            function_button.setFixedHeight(30)
            function_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            function_button.clicked.connect(lambda _, fn=function: self.perform_function(fn))
            function_layout.addWidget(function_button)
        function_group.setLayout(function_layout)
        right_column.addWidget(function_group)

        # Add DB 확인 버튼 at the bottom right
        self.db_button = QPushButton("DB 확인", self)
        self.db_button.setStyleSheet("background-color: blue; color: white; font-size: 20px;")
        self.db_button.setFixedSize(150, 50)
        self.db_button.clicked.connect(self.open_db_window)
        right_column.addStretch()  # Push the DB 버튼 to the bottom
        right_column.addWidget(self.db_button)

        main_layout.addLayout(right_column, 2)  # 비율 조정 (덜 넓은 공간 할당)

        self.setLayout(main_layout)

    def timerEvent(self, event):
        while not self.order_queue.empty():
            request, request_id = self.order_queue.get()
            self.display_order(request, request_id)
        while not self.staff_call_queue.empty():
            message = self.staff_call_queue.get()
            self.display_staff_call_popup(message)

    def display_order(self, request, request_id):
        # Get the table name
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")

        # Store current order for the table
        self.current_orders[table_name] = {'request': request, 'request_id': request_id}

        # Update the order display area
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            # Append "(수락 대기중)" to each menu item
            order_text = '\n'.join([f"{item}(수락 대기중)" for item in request.menu])
            order_details_label.setText(order_text)
        else:
            self.show_error(f"{table_name}의 주문 내역 표시 레이블을 찾을 수 없습니다.")

        # Activate the '주문 수락', '재료 부족', '하기 싫음' 버튼 for this table
        order_accept_button = self.order_accept_buttons.get(table_name)
        out_of_stock_button = self.out_of_stock_buttons.get(table_name)
        not_in_mood_button = self.not_in_mood_buttons.get(table_name)
        cooking_done_button = self.cooking_done_buttons.get(table_name)

        if order_accept_button and out_of_stock_button and not_in_mood_button and cooking_done_button:
            order_accept_button.setEnabled(True)
            out_of_stock_button.setEnabled(True)
            not_in_mood_button.setEnabled(True)
            cooking_done_button.setEnabled(False)  # 조리 완료 버튼은 주문 수락 후 활성화
        else:
            self.show_error(f"{table_name}의 버튼 중 하나 이상을 찾을 수 없습니다.")

    def accept_order(self, table_name):
        order = self.current_orders.get(table_name)
        if order:
            request = order['request']
            request_id = order['request_id']
            response = Order.Response()
            response.response = ['주문 수락']

            self.node.get_logger().info(f"{table_name}의 주문을 수락합니다.")

            # Update the response in KitchenGUINode
            self.node.update_order_response(request_id, response)

            # Update the order details to show items and quantities without "(수락 대기중)"
            order_details_label = self.order_details_labels.get(table_name)
            if order_details_label:
                order_text = '\n'.join(request.menu)
                order_details_label.setText(order_text)

            # Disable '주문 수락', '재료 부족', '하기 싫음' 버튼
            self.order_accept_buttons[table_name].setEnabled(False)
            self.out_of_stock_buttons[table_name].setEnabled(False)
            self.not_in_mood_buttons[table_name].setEnabled(False)

            # Enable '조리 완료' 버튼
            self.cooking_done_buttons[table_name].setEnabled(True)

            QMessageBox.information(self, "주문 수락", f"{table_name}의 주문이 수락되었습니다.")

            # We may keep the order in current_orders until cooking is done
        else:
            self.show_error(f"{table_name}에 처리 중인 주문이 없습니다.")

    def send_reject_reason(self, table_name, reason):
        order = self.current_orders.get(table_name)
        if order:
            request = order['request']
            request_id = order['request_id']
            response = Order.Response()
            response.response = [reason]

            self.node.get_logger().info(f"{table_name}의 주문 거절 이유: {reason}")

            # Update the response in KitchenGUINode
            self.node.update_order_response(request_id, response)

            # Disable all action buttons
            self.order_accept_buttons[table_name].setEnabled(False)
            self.out_of_stock_buttons[table_name].setEnabled(False)
            self.not_in_mood_buttons[table_name].setEnabled(False)
            self.cooking_done_buttons[table_name].setEnabled(False)

            # Clear the order details
            order_details_label = self.order_details_labels.get(table_name)
            if order_details_label:
                order_details_label.setText("주문 내역 표시 영역")

            # Send rejection message to user_gui.py
            rejection_message = f"{reason} 이유로 주문이 거절되었습니다."
            self.node.publish_rejection(table_name, rejection_message)

            QMessageBox.information(self, "주문 거절", f"{table_name}의 주문이 거절되었습니다: {reason}")

            # Remove the order from current_orders as it's rejected
            self.current_orders.pop(table_name)
        else:
            self.show_error(f"{table_name}에 처리 중인 주문이 없습니다.")

    def mark_cooking_done(self, table_name):
        self.node.get_logger().info(f"{table_name}의 조리가 완료되었습니다.")

        # Disable '조리 완료' 버튼
        self.cooking_done_buttons[table_name].setEnabled(False)

        # Clear the order details
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            order_details_label.setText("주문 내역 표시 영역")

        QMessageBox.information(self, "조리 완료", f"{table_name}의 조리가 완료되었습니다.")

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

    def show_error(self, message):
        QMessageBox.critical(self, "오류", message)

    def display_staff_call_popup(self, message):
        # Display the popup in the GUI thread
        QMessageBox.information(self, "직원 호출", message)

    def open_db_window(self):
        if self.db_window is None:
            self.db_window = DBWindow()
        self.db_window.show()


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
        ros_thread.join()


if __name__ == "__main__":
    main()
