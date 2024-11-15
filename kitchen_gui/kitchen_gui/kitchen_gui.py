# kitchen_gui/kitchen_gui.py

import sys
import threading
import queue
import sqlite3  # SQLite3 사용을 위한 임포트
import datetime  # 주문 시각을 기록하기 위한 임포트
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose  # NavigateToPose 액션 임포트
from rclpy.action import ActionClient  # ROS2 액션 클라이언트 임포트
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGroupBox, QMessageBox, QGridLayout, QButtonGroup, QSizePolicy,
    QMainWindow, QTableWidget, QTableWidgetItem, QTextEdit, QHeaderView  # QHeaderView 추가
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QTextOption  # QTextOption 임포트
from custom_interface.srv import Order  # Order 서비스 임포트
from action_msgs.srv import CancelGoal  # Import CancelGoal service


# QOS
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
qos_order = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, # 신뢰성 중시
    history=QoSHistoryPolicy.KEEP_ALL, # 모든 데이터 보관
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL # 생성되기 전 데이터 보관
)
qos_alarm = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, # 신뢰성 중시
    history=QoSHistoryPolicy.KEEP_LAST, # 정해진 메시지 큐 만큼 보관
    depth=18, # 큐가 보관할 수 있는 최대 메시지 개수
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL # 생성되기 전 데이터 보관
)

class KitchenGUINode(Node):
    def __init__(self, order_queue):
        super().__init__('kitchen_gui')
        
        self.staff_call_queue = queue.Queue()
        
        self.declare_parameter('goal_orientation_x', 0.0)
        self.declare_parameter('goal_orientation_y', 0.0)
        self.declare_parameter('goal_orientation_z', 0.0)
        self.declare_parameter('goal_orientation_w', 1.0)
        
        # Initialize the NavigateToPose action client
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.order_queue = order_queue
        
        # Initialize the emergency stop publisher
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Order 서비스 서버 생성
        self.order_service = self.create_service(
            Order,
            'send_order',
            self.handle_order_service,
            qos_profile=qos_order
        )

        # 테이블 번호와 이름 매핑 (테이블1부터 테이블9까지)
        self.table_number_to_name = {i: f'테이블{i}' for i in range(1, 10)}

        # 주문 거절 메시지를 user_gui.py로 퍼블리시하기 위한 퍼블리셔 생성
        self.rejection_publisher = self.create_publisher(String, 'order_rejections', 10)

        # 보류 중인 주문을 추적하기 위한 딕셔너리: request_id -> (Event, response)
        self.pending_orders = {}

        # pending_orders 접근을 보호하기 위한 뮤텍스
        self.pending_orders_mutex = threading.Lock()

        # 직원 호출 메시지를 구독하기 위한 서브스크라이버 생성
        self.staff_call_subscription = self.create_subscription(
            String,
            'call_staff',
            self.staff_call_callback,
            qos_profile=qos_alarm
        )

        self.table_positions = {
            '테이블1': (2.809887647628784, 1.595257043838501, 0.0),
            '테이블2': (2.769096851348877, 0.494875967502594, 0.0),
            '테이블3': (2.753706693649292, -0.5872430801391602, 0.0),
            '테이블4': (1.6649770736694336, 1.6181622743606567, 0.0),
            '테이블5': (1.6803680658340454, 0.5104005336761475, 0.0),
            '테이블6': (1.6892523765563965, -0.6097449660301208, 0.0),
            '테이블7': (0.6251729726791382, 1.5763969421386719, 0.0),
            '테이블8': (0.5726200938224792, 0.5137917399406433, 0.0),
            '테이블9': (0.57574862241745, -0.5812110900878906, 0.0),
        }

        # SQLite3 데이터베이스 초기화
        self.init_db()

    def init_db(self):
        # SQLite3 데이터베이스 연결 (파일이 없으면 생성됨)
        self.conn = sqlite3.connect('orders.db')
        self.cursor = self.conn.cursor()

        # orders 테이블 생성 (없을 경우)
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS orders (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                order_time TEXT NOT NULL,
                table_number TEXT NOT NULL,
                order_details TEXT NOT NULL,
                amount INTEGER NOT NULL
            )
        ''')
        self.conn.commit()

    def handle_order_service(self, request, response):
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")
        self.get_logger().info(f"Received order from {table_name}: {request.menu}")

        # 동기화를 위한 Event 생성
        order_event = threading.Event()

        # 고유한 request ID 생성
        request_id = id(request)  # request 객체의 id를 식별자로 사용

        # pending_orders에 Event와 response 저장
        with self.pending_orders_mutex:
            self.pending_orders[request_id] = (order_event, response)

        # GUI가 처리할 수 있도록 주문 요청과 request_id를 큐에 추가
        self.order_queue.put((request, request_id))

        # GUI 스레드가 Event를 설정할 때까지 대기 (타임아웃: 60초)
        if order_event.wait(timeout=60):
            # Event가 설정되면, GUI가 주문을 처리하고 response를 설정했음
            with self.pending_orders_mutex:
                _, response = self.pending_orders.pop(request_id)
            self.get_logger().info(f"Order from {table_name} processed with response: {response.response}")
            return response
        else:
            # 타임아웃 발생 시
            self.get_logger().warning(f"Order from {table_name} timed out")
            with self.pending_orders_mutex:
                self.pending_orders.pop(request_id, None)
            # 타임아웃을 나타내는 기본 응답 설정
            response.response = ['주문 처리 시간 초과']
            return response

    def update_order_response(self, request_id, response):
        with self.pending_orders_mutex:
            if request_id in self.pending_orders:
                order_event, response_obj = self.pending_orders[request_id]
                response_obj.response = response.response
                # Event 설정하여 서비스 콜백을 블록 해제
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
        # 메시지를 직원 호출 큐에 추가
        self.staff_call_queue.put(message)

    def send_navigate_goal(self, table_name):
        # 테이블의 위치 정보가 정의되어 있는지 확인
        if table_name not in self.table_positions:
            self.get_logger().error(f"{table_name}의 위치 정보가 정의되어 있지 않습니다.")
            return

        # 액션 서버가 준비될 때까지 대기 (최대 3초)
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 30:  # 30 * 0.1초 = 3초
                self.get_logger().warn("Navigate action server is not available.")
                return
            wait_count += 1

        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        x, y, z = self.table_positions[table_name]
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = self.get_parameter('goal_orientation_x').value
        goal_msg.pose.pose.orientation.y = self.get_parameter('goal_orientation_y').value
        goal_msg.pose.pose.orientation.z = self.get_parameter('goal_orientation_z').value
        goal_msg.pose.pose.orientation.w = self.get_parameter('goal_orientation_w').value

        # 목표 전송
        send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        send_goal_future.add_done_callback(lambda future: self.navigate_to_pose_action_goal(future, table_name))

    def navigate_to_pose_action_feedback(self, feedback_msg):
        # 피드백 처리 (선택 사항)
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Navigating... Current pose: {feedback.current_pose.pose}")

    def navigate_to_pose_action_goal(self, future, table_name):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal for {table_name}: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().info(f"Goal rejected for {table_name}")
            return

        self.get_logger().info(f"Goal accepted for {table_name}")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.navigate_to_pose_result(future, table_name))
    
    def send_navigate_goal_to_position(self, position):
        x, y, z = position
        # 액션 서버가 준비될 때까지 대기 (최대 3초)
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 30:  # 30 * 0.1초 = 3초
                self.get_logger().warn("Navigate action server is not available.")
                return
            wait_count += 1

        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = self.get_parameter('goal_orientation_x').value
        goal_msg.pose.pose.orientation.y = self.get_parameter('goal_orientation_y').value
        goal_msg.pose.pose.orientation.z = self.get_parameter('goal_orientation_z').value
        goal_msg.pose.pose.orientation.w = self.get_parameter('goal_orientation_w').value

        # 목표 전송
        send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        send_goal_future.add_done_callback(lambda future: self.navigate_to_pose_action_goal(future, "주방"))

    def navigate_to_pose_result(self, future, table_name):
        try:
            result = future.result().result
            status = future.result().status
            if status == 4:
                self.get_logger().info(f"Goal aborted for {table_name}")
            elif status == 5:
                self.get_logger().warn(f"Goal canceled for {table_name}")
            else:
                self.get_logger().info(f"Goal succeeded for {table_name}: {result}")
        except Exception as e:
            self.get_logger().error(f"Goal failed for {table_name}: {e}")

    def send_emergency_stop(self):
        msg = Bool()
        msg.data = True  # 긴급 정지 신호
        self.emergency_stop_publisher.publish(msg)
        self.get_logger().warn("긴급 정지 명령을 로봇에 전송했습니다.")
        self.cancel_navigation()

    def cancel_navigation(self):
        """
        Cancel all NavigateToPose action goals.
        """
        self.get_logger().info('Sending navigation cancel request...')
        # Create a client for the cancel goal service
        cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if not cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose Cancel Service와 연결이 되지 않습니다!')
            return

        # Create a CancelGoal request with empty goal_info to cancel all goals
        request = CancelGoal.Request()
        # Leaving goal_info empty cancels all goals

        future = cancel_client.call_async(request)
        future.add_done_callback(self.cancel_navigation_callback)

    def cancel_navigation_callback(self, future):
        """
        Callback after sending navigation cancel request.
        """
        try:
            response = future.result()
            if len(response.goals_canceling) > 0:
                self.get_logger().info('Navigation goals have been successfully cancelled.')
            else:
                self.get_logger().warn('There are no navigation goals to cancel.')
        except Exception as e:
            self.get_logger().error(f'Error occurred while cancelling navigation: {e}')

    def close(self):
        # 데이터베이스 연결 종료
        self.cursor.close()
        self.conn.close()



def ros_spin(node):
    rclpy.spin(node)


class DBWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("DB 확인")
        self.setGeometry(200, 200, 800, 600)  # 창의 크기 조정

        # 주문 내역을 표시할 테이블 위젯
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(['ID', '주문 시각', '테이블 번호', '주문 메뉴 및 수량', '금액'])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)  # 수정된 부분
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(True)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.table)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # 데이터베이스에서 주문 내역 로드
        self.load_orders()

    def load_orders(self):
        cursor = self.node.cursor
        cursor.execute('SELECT id, order_time, table_number, order_details, amount FROM orders ORDER BY id ASC')
        rows = cursor.fetchall()
        self.table.setRowCount(len(rows) + 1)  # 총계를 위한 추가 행

        total_amount = 0

        for row_idx, row_data in enumerate(rows):
            id_item = QTableWidgetItem(str(row_data[0]))
            time_item = QTableWidgetItem(row_data[1])
            table_item = QTableWidgetItem(row_data[2])
            details_item = QTableWidgetItem(row_data[3])
            amount_item = QTableWidgetItem(f"{row_data[4]}원")

            id_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            time_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            table_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            details_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            amount_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

            self.table.setItem(row_idx, 0, id_item)
            self.table.setItem(row_idx, 1, time_item)
            self.table.setItem(row_idx, 2, table_item)
            self.table.setItem(row_idx, 3, details_item)
            self.table.setItem(row_idx, 4, amount_item)

            total_amount += row_data[4]

        # 총계 행 추가
        total_label_item = QTableWidgetItem("총계")
        total_label_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
        total_label_item.setTextAlignment(Qt.AlignCenter)

        total_amount_item = QTableWidgetItem(f"{total_amount}원")
        total_amount_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
        total_amount_item.setTextAlignment(Qt.AlignCenter)

        self.table.setItem(len(rows), 3, total_label_item)
        self.table.setItem(len(rows), 4, total_amount_item)


class KitchenGUI(QWidget):
    def __init__(self, order_queue, node):
        super().__init__()
        self.order_queue = order_queue
        self.node = node
        self.setWindowTitle("주방 GUI")
        self.setGeometry(100, 100, 1600, 900)  # 창 크기 조정

        self.current_orders = {}  # 각 테이블의 현재 주문 저장

        # 주문 내역 레이블과 버튼 딕셔너리 초기화
        self.order_details_labels = {}
        self.order_accept_buttons = {}
        self.out_of_stock_buttons = {}
        self.not_in_mood_buttons = {}
        self.cooking_done_buttons = {}

        # 테이블 번호와 이름 매핑
        self.table_number_to_name = self.node.table_number_to_name

        # 직원 호출 큐 가져오기
        self.staff_call_queue = self.node.staff_call_queue

        # DB 창 초기화
        self.db_window = None

        # 메뉴 가격 정보
        self.menu_prices = {
            '피자': 12000,
            '파스타': 10000,
            '샐러드': 7000,
            '바쓰': 15000,
            '비빔밥': 8000,
            '마파두부': 9000,
            '팔보완자': 10000,
            '꼬치': 7000,
            '한식 한상': 20000,
            '들깨찜': 12000,
            '한치순대': 11000,
            '잭더맥': 13000,
            '탄산음료': 2000,
            '맥주': 5000,
            '소주': 4000,
            '와인': 15000,
        }

        self.initUI()

        # 주문 큐와 직원 호출 큐를 주기적으로 확인하기 위한 타이머 시작
        self.timer = self.startTimer(100)  # 100ms마다 체크

    def initUI(self):
        main_layout = QHBoxLayout(self)

        # 왼쪽+가운데 칼럼: 3x3 그리드로 테이블과 기능 배치
        grid_layout = QGridLayout()

        tables = [f"테이블{i}" for i in range(1, 10)]  # 테이블1부터 테이블9까지

        for i, table in enumerate(tables):
            row = i // 3
            col = i % 3

            # 각 테이블을 위한 그룹 박스 생성
            group_box = QGroupBox(table)
            group_layout = QVBoxLayout()

            # 주문 내역 레이블을 QTextEdit으로 변경하여 텍스트가 모두 표시되도록 함
            order_details = QTextEdit("주문 내역 표시 영역")
            order_details.setReadOnly(True)
            order_details.setStyleSheet("background-color: #003366; color: white;")
            order_details.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            order_details.setWordWrapMode(QTextOption.WordWrap)  # 수정된 부분
            group_layout.addWidget(order_details)

            # 제어 버튼 레이아웃
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

            # 버튼을 해당 테이블에 연결
            order_accept_button.clicked.connect(lambda _, tb=table: self.accept_order(tb))
            out_of_stock_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "재료 부족"))
            not_in_mood_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "하기 싫음"))
            cooking_done_button.clicked.connect(lambda _, tb=table: self.mark_cooking_done(tb))

            group_layout.addLayout(buttons_layout)

            group_box.setLayout(group_layout)
            group_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            grid_layout.addWidget(group_box, row, col)

            # 주문 내역 레이블 저장
            self.order_details_labels[table] = order_details

        main_layout.addLayout(grid_layout, 5)  # 그리드에 더 넓은 공간 할당

        # 오른쪽 칼럼: 수동 조종 및 DB 확인/초기화 버튼
        right_column = QVBoxLayout()
        control_label = QLabel("수동 조종")
        control_label.setAlignment(Qt.AlignCenter)
        control_label.setFixedHeight(30)
        control_label.setStyleSheet("font-weight: bold; font-size: 20px;")
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

        # 오른쪽 칼럼 맨 아래에 DB 확인 버튼 및 DB 초기화 버튼 추가
        db_buttons_layout = QHBoxLayout()

        self.db_button = QPushButton("DB 확인")
        self.db_button.setStyleSheet("background-color: blue; color: white; font-size: 20px;")
        self.db_button.setFixedSize(150, 50)
        self.db_button.clicked.connect(self.open_db_window)
        db_buttons_layout.addWidget(self.db_button)

        self.reset_db_button = QPushButton("DB 초기화")  # 새로 추가된 버튼
        self.reset_db_button.setStyleSheet("background-color: red; color: white; font-size: 20px;")
        self.reset_db_button.setFixedSize(150, 50)
        self.reset_db_button.clicked.connect(self.reset_db)  # 버튼 기능 연결
        db_buttons_layout.addWidget(self.reset_db_button)

        right_column.addStretch()  # DB 버튼을 맨 아래로 밀기 위해 스트레치 추가
        right_column.addLayout(db_buttons_layout)  # DB 버튼들을 수평으로 배치

        main_layout.addLayout(right_column, 2)  # 오른쪽 칼럼에 덜 넓은 공간 할당

        self.setLayout(main_layout)

    def timerEvent(self, event):
        while not self.order_queue.empty():
            request, request_id = self.order_queue.get()
            self.display_order(request, request_id)
        while not self.staff_call_queue.empty():
            message = self.staff_call_queue.get()
            self.display_staff_call_popup(message)

    def display_order(self, request, request_id):
        # 테이블 이름 가져오기
        table_name = self.table_number_to_name.get(request.table_number, f"테이블{request.table_number}")

        # 테이블의 현재 주문 저장
        self.current_orders[table_name] = {'request': request, 'request_id': request_id}

        # 주문 내역 표시 영역 업데이트
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            # 각 메뉴 항목에 "(수락 대기중)" 추가
            order_text = '\n'.join([f"{item}(수락 대기중)" for item in request.menu])
            order_details_label.setPlainText(order_text)  # setText 대신 setPlainText 사용
        else:
            self.show_error(f"{table_name}의 주문 내역 표시 레이블을 찾을 수 없습니다.")

        # 해당 테이블의 '주문 수락', '재료 부족', '하기 싫음' 버튼 활성화
        order_accept_button = self.order_accept_buttons.get(table_name)
        out_of_stock_button = self.out_of_stock_buttons.get(table_name)
        not_in_mood_button = self.not_in_mood_buttons.get(table_name)
        cooking_done_button = self.cooking_done_buttons.get(table_name)

        if order_accept_button and out_of_stock_button and not_in_mood_button and cooking_done_button:
            order_accept_button.setEnabled(True)
            out_of_stock_button.setEnabled(True)
            not_in_mood_button.setEnabled(True)
            cooking_done_button.setEnabled(False)  # 주문 수락 후 조리 완료 버튼 활성화
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

            # KitchenGUINode에서 응답 업데이트
            self.node.update_order_response(request_id, response)

            # 주문 내역을 "(수락 대기중)" 없이 표시
            order_details_label = self.order_details_labels.get(table_name)
            if order_details_label:
                order_text = '\n'.join(request.menu)
                order_details_label.setPlainText(order_text)  # setText 대신 setPlainText 사용

            # '주문 수락', '재료 부족', '하기 싫음' 버튼 비활성화
            self.order_accept_buttons[table_name].setEnabled(False)
            self.out_of_stock_buttons[table_name].setEnabled(False)
            self.not_in_mood_buttons[table_name].setEnabled(False)

            # '조리 완료' 버튼 활성화
            self.cooking_done_buttons[table_name].setEnabled(True)

            QMessageBox.information(self, "주문 수락", f"{table_name}의 주문이 수락되었습니다.")

            # 주문 정보를 데이터베이스에 저장
            self.save_order_to_db(request, table_name)

            # 필요에 따라 조리가 완료될 때까지 주문을 current_orders에 유지
        else:
            self.show_error(f"{table_name}에 처리 중인 주문이 없습니다.")

    def save_order_to_db(self, request, table_name):
        # 주문 시각
        order_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 주문 메뉴 및 수량 문자열 생성
        order_details = '\n'.join(request.menu)

        # 금액 계산
        total_amount = 0
        for item in request.menu:
            if ':' in item:
                menu_item, quantity_str = item.split(':')
                try:
                    quantity = int(quantity_str)
                except ValueError:
                    quantity = 1  # 수량이 잘못된 경우 기본값 1
            elif ' x ' in item:
                menu_item, quantity_str = item.split(' x ')
                try:
                    quantity = int(quantity_str)
                except ValueError:
                    quantity = 1
            else:
                menu_item = item
                quantity = 1
            price = self.menu_prices.get(menu_item.strip(), 0)  # 메뉴 이름의 공백 제거
            subtotal = price * quantity
            total_amount += subtotal
            self.node.get_logger().info(f"Menu: {menu_item.strip()}, Quantity: {quantity}, Price: {price}, Subtotal: {subtotal}원")
            print(f"Menu: {menu_item.strip()}, Quantity: {quantity}, Price: {price}, Subtotal: {subtotal}원")

        self.node.get_logger().info(f"Total Amount: {total_amount}원")
        print(f"Total Amount: {total_amount}원")

        # 데이터베이스에 주문 삽입
        self.node.cursor.execute('INSERT INTO orders (order_time, table_number, order_details, amount) VALUES (?, ?, ?, ?)',
                                 (order_time, table_name, order_details, total_amount))
        self.node.conn.commit()

    def send_reject_reason(self, table_name, reason):
        order = self.current_orders.get(table_name)
        if order:
            request = order['request']
            request_id = order['request_id']
            response = Order.Response()
            response.response = [reason]

            self.node.get_logger().info(f"{table_name}의 주문 거절 이유: {reason}")

            # KitchenGUINode에서 응답 업데이트
            self.node.update_order_response(request_id, response)

            # 모든 액션 버튼 비활성화
            self.order_accept_buttons[table_name].setEnabled(False)
            self.out_of_stock_buttons[table_name].setEnabled(False)
            self.not_in_mood_buttons[table_name].setEnabled(False)
            self.cooking_done_buttons[table_name].setEnabled(False)

            # 주문 내역 초기화
            order_details_label = self.order_details_labels.get(table_name)
            if order_details_label:
                order_details_label.setPlainText("주문 내역 표시 영역")  # setText 대신 setPlainText 사용

            # user_gui.py로 거절 메시지 전송
            rejection_message = f"{reason} 이유로 주문이 거절되었습니다."
            self.node.publish_rejection(table_name, rejection_message)

            QMessageBox.information(self, "주문 거절", f"{table_name}의 주문이 거절되었습니다: {reason}")

            # 거절된 주문을 current_orders에서 제거
            self.current_orders.pop(table_name)
        else:
            self.show_error(f"{table_name}에 처리 중인 주문이 없습니다.")

    def mark_cooking_done(self, table_name):
        self.node.get_logger().info(f"{table_name}의 조리가 완료되었습니다.")

        # '조리 완료' 버튼 비활성화
        self.cooking_done_buttons[table_name].setEnabled(False)

        # 주문 내역 초기화
        order_details_label = self.order_details_labels.get(table_name)
        if order_details_label:
            order_details_label.setPlainText("주문 내역 표시 영역")  # setText 대신 setPlainText 사용

        QMessageBox.information(self, "조리 완료", f"{table_name}의 조리가 완료되었습니다.")

        # 로봇에게 네비게이션 목표 전송
        self.node.send_navigate_goal(table_name)

        # 주문을 current_orders에서 제거
        self.current_orders.pop(table_name, None)

    def select_table(self, table):
        self.selected_table = table if self.selected_table != table else None
        for button in self.table_button_group.buttons():
            button.setStyleSheet("background-color: yellow;" if button.text() == self.selected_table else "")

    def perform_function(self, function):
        if function == "긴급 정지":
            self.node.get_logger().info(f"{function} 기능을 수행합니다.")
            QMessageBox.information(self, "기능 수행", f"{function} 기능을 수행합니다.")
            # 긴급 정지 기능 구현 코드 추가
            self.node.send_emergency_stop()
        elif function == "주방 복귀":
            self.node.get_logger().info(f"{function} 기능을 수행합니다.")
            QMessageBox.information(self, "기능 수행", f"{function} 기능을 수행합니다.")
            # 주방 좌표로 로봇을 이동시키는 코드 추가
            kitchen_position = (-0.007121707778424025, -0.017804037779569626, 0.0)  # 주방의 좌표를 실제 값으로 설정하세요.
            self.node.send_navigate_goal_to_position(kitchen_position)
        elif function == "로봇 보내기":
            if self.selected_table:
                self.node.get_logger().info(f"{self.selected_table}에서 {function} 기능을 수행합니다.")
                QMessageBox.information(self, "기능 수행", f"{self.selected_table}에서 {function} 기능을 수행합니다.")
                # 선택된 테이블에 대해 기능을 수행하는 코드
                self.node.send_navigate_goal(self.selected_table)
            else:
                self.node.get_logger().info("선택된 테이블이 없습니다.")
                QMessageBox.warning(self, "경고", "선택된 테이블이 없습니다.")
        else:
            self.node.get_logger().info("작동 오류!")
            QMessageBox.warning(self, "경고", "작동 오류!")

    def show_error(self, message):
        QMessageBox.critical(self, "오류", message)

    def display_staff_call_popup(self, message):
        # GUI 스레드에서 팝업 표시
        QMessageBox.information(self, "직원 호출", message)

    def open_db_window(self):
        # 디버깅을 위해 로그 또는 print 추가
        self.node.get_logger().info("DB 확인 버튼이 클릭되었습니다.")
        print("DB 확인 버튼이 클릭되었습니다.")

        try:
            if self.db_window is None:
                self.db_window = DBWindow(self.node)
            else:
                # 데이터 새로 고침
                self.db_window.load_orders()
            self.db_window.show()
        except Exception as e:
            self.node.get_logger().error(f"DB 창 열기 실패: {e}")
            QMessageBox.critical(self, "오류", f"DB 창을 여는 중 오류가 발생했습니다: {e}")

    def reset_db(self):
        # DB 초기화 버튼 기능 구현
        reply = QMessageBox.question(
            self,
            "DB 초기화",
            "데이터베이스를 초기화하면 모든 주문 기록이 삭제됩니다. 정말로 초기화하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            try:
                # 모든 주문 기록 삭제
                self.node.cursor.execute('DELETE FROM orders')
                # AUTOINCREMENT 초기화
                self.node.cursor.execute("DELETE FROM sqlite_sequence WHERE name='orders'")
                self.node.conn.commit()
                self.node.get_logger().info("데이터베이스가 초기화되었습니다.")
                QMessageBox.information(self, "DB 초기화", "데이터베이스가 성공적으로 초기화되었습니다.")

                # DB 창이 열려있다면 새로 고침
                if self.db_window and self.db_window.isVisible():
                    self.db_window.load_orders()

            except Exception as e:
                self.node.get_logger().error(f"데이터베이스 초기화 실패: {e}")
                QMessageBox.critical(self, "오류", f"데이터베이스 초기화 중 오류가 발생했습니다: {e}")

    def closeEvent(self, event):
        # 타이머 중지
        self.killTimer(self.timer)

        # 데이터베이스 연결 종료
        self.node.close()

        event.accept()

def main(args=None):
    rclpy.init(args=args)

    order_queue = queue.Queue()
    node = KitchenGUINode(order_queue)

    # ROS2 스피닝을 별도의 스레드에서 실행
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
