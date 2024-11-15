import sys
import time
import threading
import os
import queue
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QMessageBox, QStackedWidget, QMainWindow, QSpinBox, QScrollArea,
    QSpacerItem, QSizePolicy, QGridLayout, QGroupBox
)
from PyQt5.QtGui import QPixmap, QIcon, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QObject, QEvent, QSize

import rclpy
from rclpy.node import Node
from custom_interface.srv import Order  # Import Order service
from std_msgs.msg import String

# Define the absolute path to the images directory
IMAGE_PATH = "/home/theo/2_ws/src/user_gui/images"  # <-- Update to your actual images directory path


class StaffCallThread(QThread):
    call_started = pyqtSignal()
    call_completed = pyqtSignal()

    def run(self):
        self.call_started.emit()
        time.sleep(2)
        self.call_completed.emit()


class InactivityEventFilter(QObject):
    def __init__(self, reset_callback):
        super().__init__()
        self.reset_callback = reset_callback

    def eventFilter(self, obj, event):
        if event.type() in [QEvent.MouseMove, QEvent.KeyPress, QEvent.MouseButtonPress]:
            self.reset_callback()
        return False


class OrderRejectionSubscriber(QObject):
    rejection_received = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.subscription = self.node.create_subscription(
            String,
            'order_rejections',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        rejection_message = msg.data  # "테이블A: 재료 부족 이유로 주문이 거절되었습니다."
        self.node.get_logger().info(f"Received rejection message: {rejection_message}")
        # Extract the message part after the colon and space
        try:
            _, message = rejection_message.split(": ", 1)
        except ValueError:
            message = rejection_message
        # Emit the signal with the message
        self.rejection_received.emit(message)


class RestaurantRobotGUI(QMainWindow):
    # Define signals to handle order responses and waiting state
    order_response_signal = pyqtSignal(object)
    order_waiting_signal = pyqtSignal()

    def __init__(self, node, table_number, table_name):
        super().__init__()
        self.node = node  # ROS2 node

        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d4f4dd,      /* 상단 색상 */
                    stop:1 #ffe4e1       /* 하단 색상 */
                );
            }
        """)

        # Set table information from parameters
        self.table_number = table_number
        self.table_name = table_name

        self.setWindowTitle("식당 서비스 로봇")
        self.setGeometry(100, 100, 800, 600)  # 창 크기를 키움

        self.cart = {}
        self.menu = {
            "피자": ["pizza.png", 12000],
            "파스타": ["pasta.png", 10000],
            "샐러드": ["salad.png", 7000],
            "콜라": ["cola.png", 2000]
        }

        # Create service client instead of publisher
        self.send_order_client = self.node.create_client(Order, 'send_order')
        self.call_staff_publisher = self.node.create_publisher(String, 'call_staff', 10)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.waiting_screen = self.create_waiting_screen()
        self.menu_screen = self.create_menu_screen()
        self.cart_screen = self.create_cart_screen()

        self.stack.addWidget(self.waiting_screen)
        self.stack.addWidget(self.menu_screen)
        self.stack.addWidget(self.cart_screen)

        self.staff_call_thread = StaffCallThread()
        self.staff_call_thread.call_started.connect(self.on_staff_call_started)
        self.staff_call_thread.call_completed.connect(self.on_staff_call_completed)

        self.inactivity_timer = QTimer()
        self.inactivity_timer.setInterval(10000)  # 10 seconds
        self.inactivity_timer.setSingleShot(True)
        self.inactivity_timer.timeout.connect(self.return_to_waiting_screen)

        self.inactivity_event_filter = InactivityEventFilter(self.reset_inactivity_timer)
        self.menu_screen.installEventFilter(self.inactivity_event_filter)

        self.stack.currentChanged.connect(self.on_screen_changed)

        # Connect the order response signal
        self.order_response_signal.connect(self.handle_order_response)
        self.order_waiting_signal.connect(self.show_order_waiting_popup)

        # Initialize the order rejection subscriber
        self.order_rejection_subscriber = OrderRejectionSubscriber(self.node)
        self.order_rejection_subscriber.rejection_received.connect(self.show_rejection_popup)

        # Flag to prevent multiple waiting popups
        self.order_waiting_popup_shown = False

    def create_waiting_screen(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        layout.setContentsMargins(20, 20, 20, 20)  # 여백 조정
        layout.setSpacing(10)
        widget.setStyleSheet("""
            QWidget {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #d4f4dd,      /* 좌측 색상 */
                    stop:1 #ffe4e1       /* 우측 색상 */
                );
            }
        """)
        logo_label = QLabel()
        logo_path = os.path.join(IMAGE_PATH, 'logo.png')  # Absolute path
        if not os.path.exists(logo_path):
            QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 파일을 찾을 수 없습니다: {logo_path}")
        else:
            logo_pixmap = QPixmap(logo_path)
            if logo_pixmap.isNull():
                QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 로딩에 실패했습니다: {logo_path}")
            else:
                logo_pixmap = logo_pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                logo_label.setPixmap(logo_pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)

        label = QLabel("환영합니다!\n'주문하기'를 눌러주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(label)

        self.staff_call_status_label = QLabel("")
        self.staff_call_status_label.setAlignment(Qt.AlignCenter)
        self.staff_call_status_label.setStyleSheet("font-size: 14px; color: blue;")
        layout.addWidget(self.staff_call_status_label)

        order_button = QPushButton("주문하기")
        order_button.setFixedSize(250, 60)
        order_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4CAF50,
                    stop:1 #45a049
                );
                color: white;
                border-radius: 15px;
                font-size: 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #45a049,
                    stop:1 #3e8e41
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3e8e41,
                    stop:1 #367c39
                );
            }
        """)
        order_button.clicked.connect(self.show_menu_screen)
        layout.addWidget(order_button, alignment=Qt.AlignCenter)

        call_staff_button = QPushButton("직원 호출")
        call_staff_button.setFixedSize(250, 60)
        call_staff_icon_path = os.path.join(IMAGE_PATH, 'call_staff.png')  # Absolute path
        if not os.path.exists(call_staff_icon_path):
            QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 파일을 찾을 수 없습니다: {call_staff_icon_path}")
        else:
            call_staff_pixmap = QPixmap(call_staff_icon_path)
            if call_staff_pixmap.isNull():
                QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 로딩에 실패했습니다: {call_staff_icon_path}")
            else:
                call_staff_icon = QIcon(call_staff_pixmap.scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                call_staff_button.setIcon(call_staff_icon)
                call_staff_button.setIconSize(call_staff_pixmap.scaled(40, 40, Qt.KeepAspectRatio, Qt.SmoothTransformation).size())
        call_staff_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f44336,
                    stop:1 #da190b
                );
                color: white;
                border-radius: 15px;
                font-size: 20px;
                font-weight: bold;
                padding-left: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #da190b,
                    stop:1 #b71c1c
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #b71c1c,
                    stop:1 #9c1313
                );
            }
        """)
        call_staff_button.clicked.connect(self.call_staff_topic)
        layout.addWidget(call_staff_button, alignment=Qt.AlignCenter)

        layout.addSpacerItem(QSpacerItem(0, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        widget.setLayout(layout)
        return widget

    def create_menu_screen(self):
        widget = QWidget()
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignTop)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        label = QLabel("메뉴를 선택하세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 28px; font-weight: bold;")
        main_layout.addWidget(label)

        grid_layout = QGridLayout()
        grid_layout.setSpacing(30)  # 메뉴 항목 간의 간격 조정

        self.quantity_spinboxes = {}

        row = 0
        column = 0
        max_columns = 2  # 한 행에 표시할 최대 열 수

        for idx, (item, (image_filename, price)) in enumerate(self.menu.items()):
            group_box = QGroupBox()
            group_box.setStyleSheet("""
                QGroupBox {
                    border: 2px solid #dcdcdc;
                    border-radius: 15px;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    padding: 0 3px 0 3px;
                    font-size: 16px;
                }
            """)
            group_layout = QVBoxLayout()
            group_layout.setAlignment(Qt.AlignCenter)
            group_layout.setSpacing(10)

            # 메뉴 이미지
            image_label = QLabel()
            image_path = os.path.join(IMAGE_PATH, image_filename)  # 절대 경로
            if not os.path.exists(image_path):
                QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 파일을 찾을 수 없습니다: {image_path}")
                continue
            else:
                pixmap = QPixmap(image_path)
                if pixmap.isNull():
                    QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 로딩에 실패했습니다: {image_path}")
                    continue
                else:
                    pixmap = pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            group_layout.addWidget(image_label)

            # 메뉴 이름 및 가격
            name_label = QLabel(f"{item}")
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet("font-size: 20px; font-weight: bold;")
            group_layout.addWidget(name_label)

            price_label = QLabel(f"{price}원")
            price_label.setAlignment(Qt.AlignCenter)
            price_label.setStyleSheet("font-size: 18px; color: #555555;")
            group_layout.addWidget(price_label)

            # 수량 선택
            spin_box = QSpinBox()
            spin_box.setRange(0, 10)
            spin_box.setValue(0)
            spin_box.setFixedWidth(80)
            spin_box.setStyleSheet("""
                QSpinBox {
                    font-size: 16px;
                    padding: 5px;
                }
            """)
            self.quantity_spinboxes[item] = spin_box
            group_layout.addWidget(spin_box, alignment=Qt.AlignCenter)

            # 추가 버튼 (장바구니에 추가)
            add_button = QPushButton("장바구니에 추가")
            add_button.setFixedSize(180, 50)
            add_button.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(
                        x1:0, y1:0, x2:0, y2:1,
                        stop:0 #4CAF50,
                        stop:1 #45a049
                    );
                    color: white;
                    border-radius: 15px;
                    font-size: 16px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:0, y2:1,
                        stop:0 #45a049,
                        stop:1 #3e8e41
                    );
                }
                QPushButton:pressed {
                    background: qlineargradient(
                        x1:0, y1:0, x2:0, y2:1,
                        stop:0 #3e8e41,
                        stop:1 #367c39
                    );
                }
            """)
            add_button.clicked.connect(lambda _, i=item: self.add_to_cart(i))
            group_layout.addWidget(add_button)

            group_box.setLayout(group_layout)
            grid_layout.addWidget(group_box, row, column)

            column += 1
            if column >= max_columns:
                column = 0
                row += 1

        main_layout.addLayout(grid_layout)

        # 하단 버튼들 (직원 호출, 장바구니 보기, 초기 화면으로 돌아가기)
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(30)

        # 직원 호출 버튼
        call_staff_button = QPushButton("직원 호출")
        call_staff_button.setFixedSize(200, 60)
        call_staff_icon_path = os.path.join(IMAGE_PATH, 'call_staff.png')  # 절대 경로
        if not os.path.exists(call_staff_icon_path):
            QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 파일을 찾을 수 없습니다: {call_staff_icon_path}")
        else:
            call_staff_pixmap = QPixmap(call_staff_icon_path)
            if call_staff_pixmap.isNull():
                QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 로딩에 실패했습니다: {call_staff_icon_path}")
            else:
                call_staff_icon = QIcon(call_staff_pixmap.scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                call_staff_button.setIcon(call_staff_icon)
                call_staff_button.setIconSize(QSize(30, 30))
        call_staff_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f44336,
                    stop:1 #da190b
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
                padding-left: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #da190b,
                    stop:1 #b71c1c
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #b71c1c,
                    stop:1 #9c1313
                );
            }
        """)
        call_staff_button.clicked.connect(self.call_staff_topic)
        buttons_layout.addWidget(call_staff_button)

        # 장바구니 보기 버튼
        cart_button = QPushButton("장바구니 보기")
        cart_button.setFixedSize(200, 60)
        cart_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FF9800,
                    stop:1 #FB8C00
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FB8C00,
                    stop:1 #F57C00
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #F57C00,
                    stop:1 #EF6C00
                );
            }
        """)
        cart_button.clicked.connect(self.show_cart)
        buttons_layout.addWidget(cart_button)

        # 초기 화면으로 돌아가기 버튼
        back_to_home_button = QPushButton("초기 화면으로 돌아가기")
        back_to_home_button.setFixedSize(250, 60)
        back_to_home_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #9E9E9E,
                    stop:1 #757575
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #757575,
                    stop:1 #616161
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #616161,
                    stop:1 #424242
                );
            }
        """)
        back_to_home_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.waiting_screen))
        buttons_layout.addWidget(back_to_home_button)

        main_layout.addLayout(buttons_layout)

        widget.setLayout(main_layout)
        return widget

    def create_cart_screen(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        widget.setStyleSheet("""
            QWidget {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #ffe4e1,
                    stop:1 #d4f4dd
                );
            }
        """)
        label = QLabel("장바구니")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 28px; font-weight: bold;")
        layout.addWidget(label)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        self.cart_items_layout = QVBoxLayout(scroll_content)
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        self.total_label = QLabel("총 금액: 0원")
        self.total_label.setAlignment(Qt.AlignCenter)
        self.total_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(self.total_label)

        self.cart_staff_call_status_label = QLabel("")
        self.cart_staff_call_status_label.setAlignment(Qt.AlignCenter)
        self.cart_staff_call_status_label.setStyleSheet("font-size: 14px; color: blue;")
        layout.addWidget(self.cart_staff_call_status_label)

        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(30)

        pay_button = QPushButton("결제하기")
        pay_button.setFixedSize(200, 60)
        pay_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #673AB7,
                    stop:1 #512DA8
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #512DA8,
                    stop:1 #4527A0
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4527A0,
                    stop:1 #311B92
                );
            }
        """)
        pay_button.clicked.connect(self.pay)
        buttons_layout.addWidget(pay_button)

        clear_button = QPushButton("장바구니 초기화")
        clear_button.setFixedSize(200, 60)
        clear_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #E91E63,
                    stop:1 #C2185B
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #C2185B,
                    stop:1 #AD1457
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #AD1457,
                    stop:1 #880E4F
                );
            }
        """)
        clear_button.clicked.connect(self.clear_cart)
        buttons_layout.addWidget(clear_button)

        back_button = QPushButton("메뉴로 돌아가기")
        back_button.setFixedSize(200, 60)
        back_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #607D8B,
                    stop:1 #455A64
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #455A64,
                    stop:1 #37474F
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #37474F,
                    stop:1 #263238
                );
            }
        """)
        back_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.menu_screen))
        buttons_layout.addWidget(back_button)

        cart_call_staff_button = QPushButton("직원 호출")
        cart_call_staff_button.setFixedSize(200, 60)
        cart_call_staff_icon_path = os.path.join(IMAGE_PATH, 'call_staff.png')  # Absolute path
        if not os.path.exists(cart_call_staff_icon_path):
            QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 파일을 찾을 수 없습니다: {cart_call_staff_icon_path}")
        else:
            cart_call_staff_pixmap = QPixmap(cart_call_staff_icon_path)
            if cart_call_staff_pixmap.isNull():
                QMessageBox.critical(self, "이미지 로딩 오류", f"이미지 로딩에 실패했습니다: {cart_call_staff_icon_path}")
            else:
                cart_call_staff_icon = QIcon(cart_call_staff_pixmap.scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                cart_call_staff_button.setIcon(cart_call_staff_icon)
                cart_call_staff_button.setIconSize(QSize(30, 30))
        cart_call_staff_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f44336,
                    stop:1 #da190b
                );
                color: white;
                border-radius: 15px;
                font-size: 18px;
                font-weight: bold;
                padding-left: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #da190b,
                    stop:1 #b71c1c
                );
            }
            QPushButton:pressed {
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #b71c1c,
                    stop:1 #9c1313
                );
            }
        """)
        cart_call_staff_button.clicked.connect(self.call_staff_topic)
        buttons_layout.addWidget(cart_call_staff_button)

        layout.addLayout(buttons_layout)

        widget.setLayout(layout)
        return widget

    def show_menu_screen(self):
        self.stack.setCurrentWidget(self.menu_screen)
        self.inactivity_timer.start()

    def on_screen_changed(self, index):
        current_widget = self.stack.widget(index)
        if current_widget == self.menu_screen:
            self.inactivity_timer.start()
        else:
            self.inactivity_timer.stop()

    def reset_inactivity_timer(self):
        if self.stack.currentWidget() == self.menu_screen:
            self.inactivity_timer.start()

    def return_to_waiting_screen(self):
        QMessageBox.information(self, "시간 초과", "10초 동안 활동이 없어 대기 화면으로 돌아갑니다.")
        self.stack.setCurrentWidget(self.waiting_screen)

    def show_cart(self):
        self.update_cart_screen()
        self.stack.setCurrentWidget(self.cart_screen)

    def update_cart_screen(self):
        while self.cart_items_layout.count():
            item = self.cart_items_layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()

        total = 0
        for item, quantity in self.cart.items():
            if quantity > 0:
                item_widget = QWidget()
                item_layout = QHBoxLayout()
                item_layout.setAlignment(Qt.AlignCenter)

                item_label = QLabel(f"{item} - {self.menu[item][1]}원 x {quantity}개")
                item_label.setAlignment(Qt.AlignCenter)
                item_label.setStyleSheet("font-size: 18px;")
                item_layout.addWidget(item_label)

                spin_box = QSpinBox()
                spin_box.setRange(1, 10)
                spin_box.setValue(quantity)
                spin_box.setFixedWidth(60)
                spin_box.setStyleSheet("""
                    QSpinBox {
                        font-size: 16px;
                        padding: 5px;
                    }
                """)
                spin_box.valueChanged.connect(lambda val, i=item: self.change_quantity(i, val))
                item_layout.addWidget(spin_box)

                delete_button = QPushButton("삭제")
                delete_button.setFixedSize(80, 40)
                delete_button.setStyleSheet("""
                    QPushButton {
                        background: qlineargradient(
                            x1:0, y1:0, x2:1, y2:1,
                            stop:0 #FF5722,
                            stop:1 #E64A19
                        );
                        color: white;
                        border-radius: 10px;
                        font-size: 14px;
                        font-weight: bold;
                    }
                    QPushButton:hover {
                        background: qlineargradient(
                            x1:0, y1:0, x2:1, y2:1,
                            stop:0 #E64A19,
                            stop:1 #D84315
                        );
                    }
                    QPushButton:pressed {
                        background: qlineargradient(
                            x1:0, y1:0, x2:1, y2:1,
                            stop:0 #D84315,
                            stop:1 #BF360C
                        );
                    }
                """)
                delete_button.clicked.connect(lambda _, i=item: self.remove_from_cart(i))
                item_layout.addWidget(delete_button)

                item_widget.setLayout(item_layout)
                self.cart_items_layout.addWidget(item_widget)

                total += self.menu[item][1] * quantity

        self.total_label.setText(f"총 금액: {total}원")

    def clear_cart(self):
        self.cart.clear()
        QMessageBox.information(self, "장바구니", "장바구니가 초기화되었습니다.")
        self.update_cart_screen()

    def add_to_cart(self, item):
        quantity = self.quantity_spinboxes[item].value()
        if quantity > 0:
            if item in self.cart:
                self.cart[item] += quantity
            else:
                self.cart[item] = quantity
            QMessageBox.information(
                self, "장바구니",
                f"{item}이(가) {quantity}개 장바구니에 추가되었습니다."
            )
            self.update_cart_screen()
            self.reset_inactivity_timer()
        else:
            QMessageBox.warning(self, "수량 오류", "1개 이상 선택해야 합니다.")

    def change_quantity(self, item, quantity):
        if quantity > 0:
            self.cart[item] = quantity
        else:
            del self.cart[item]
        self.update_cart_screen()
        self.reset_inactivity_timer()

    def remove_from_cart(self, item):
        if item in self.cart:
            del self.cart[item]
            QMessageBox.information(
                self, "장바구니",
                f"{item}이(가) 장바구니에서 삭제되었습니다."
            )
            self.update_cart_screen()
            self.reset_inactivity_timer()

    def pay(self):
        if not self.cart or all(q == 0 for q in self.cart.values()):
            QMessageBox.warning(self, "결제 오류", "장바구니가 비어 있습니다.")
            return

        # Prepare order data
        order_menu = [f"{item}:{qty}" for item, qty in self.cart.items()]

        # Get current time
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        self.send_order_service(order_menu, current_time)

    def send_order_service(self, order_menu, current_time):
        if not self.send_order_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('SendOrder 서비스가 준비되지 않았습니다.')
            QMessageBox.warning(self, "서비스 오류", "SendOrder 서비스가 준비되지 않았습니다.")
            return

        request = Order.Request()
        request.table_number = self.table_number  # Current table number
        request.time = [current_time]
        request.menu = order_menu

        # Show waiting popup
        self.order_waiting_signal.emit()

        future = self.send_order_client.call_async(request)
        future.add_done_callback(self.send_order_response)

    def show_order_waiting_popup(self):
        if not self.order_waiting_popup_shown:
            QMessageBox.information(self, "주문 전송", "수락 대기중입니다.")
            self.order_waiting_popup_shown = True

    def send_order_response(self, future):
        try:
            response = future.result()
            # Reset the waiting popup flag
            self.order_waiting_popup_shown = False
            # Emit the signal to update GUI in the main thread
            self.order_response_signal.emit(response)
        except Exception as e:
            self.order_response_signal.emit(e)

    def handle_order_response(self, response):
        if isinstance(response, Exception):
            self.node.get_logger().error('주문 전송 실패: %r' % (response,))
            QMessageBox.critical(self, "주문 전송", f"주문 전송 중 오류가 발생했습니다: {response}")
            self.order_waiting_popup_shown = False  # Reset the flag
        else:
            # Handle the successful response
            self.node.get_logger().info(f"주문 전송 결과: {', '.join(response.response)}")
            if '주문 수락' in response.response:
                QMessageBox.information(self, "주문 수락", "주문이 수락되었습니다!")
                self.cart.clear()
                self.update_cart_screen()
                self.stack.setCurrentWidget(self.waiting_screen)
            elif '주문 처리 시간 초과' in response.response:
                QMessageBox.warning(self, "주문 처리 시간 초과", "주문 처리 시간이 초과되었습니다. 다시 시도해주세요.")
                self.cart.clear()
                self.update_cart_screen()
                self.stack.setCurrentWidget(self.menu_screen)
            else:
                QMessageBox.warning(self, "주문 거절", f"주문이 거절되었습니다: {', '.join(response.response)}")
                self.cart.clear()
                self.update_cart_screen()
                self.stack.setCurrentWidget(self.menu_screen)
                # 사용자에게 다시 주문할 수 있도록 메뉴 화면으로 이동

    def call_staff_topic(self):
        # Publish a string message indicating a staff call request with table name
        message = String()
        message.data = f"테이블{self.table_number}에서 직원을 호출하였습니다."
        self.call_staff_publisher.publish(message)

        # Start the staff call thread to simulate call status
        if not self.staff_call_thread.isRunning():
            self.staff_call_thread.start()

    def on_staff_call_started(self):
        current_widget = self.stack.currentWidget()
        if current_widget == self.waiting_screen:
            self.staff_call_status_label.setText("직원 호출 중...")
        elif current_widget == self.cart_screen:
            self.cart_staff_call_status_label.setText("직원 호출 중...")
        self.disable_staff_call_buttons()

    def on_staff_call_completed(self):
        current_widget = self.stack.currentWidget()
        if current_widget == self.waiting_screen:
            self.staff_call_status_label.setText("호출완료")
        elif current_widget == self.cart_screen:
            self.cart_staff_call_status_label.setText("호출완료")
        self.enable_staff_call_buttons()
        QMessageBox.information(self, "직원 호출", "호출완료")

    def disable_staff_call_buttons(self):
        for screen in [self.waiting_screen, self.menu_screen, self.cart_screen]:
            call_buttons = [btn for btn in screen.findChildren(QPushButton) if btn.text() == "직원 호출"]
            for button in call_buttons:
                button.setEnabled(False)

    def enable_staff_call_buttons(self):
        for screen in [self.waiting_screen, self.menu_screen, self.cart_screen]:
            call_buttons = [btn for btn in screen.findChildren(QPushButton) if btn.text() == "직원 호출"]
            for button in call_buttons:
                button.setEnabled(True)

    def show_rejection_popup(self, message):
        self.node.get_logger().info(f"Displaying rejection popup: {message}")
        QMessageBox.information(self, "주문 거절", message)
        # 주문 거절 시 메뉴 화면으로 이동하여 다시 주문할 수 있도록 함
        self.stack.setCurrentWidget(self.menu_screen)

    def closeEvent(self, event):
        if self.staff_call_thread.isRunning():
            self.staff_call_thread.terminate()
            self.staff_call_thread.wait()
        event.accept()


def ros2_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    node = Node('user_gui_node')

    # 'table' 파라미터를 선언하고 기본값을 1로 설정합니다.
    node.declare_parameter('table', 1)

    # 'table' 파라미터 값을 가져옵니다.
    table_param = node.get_parameter('table').value
    table_number = int(table_param)  # 정수로 변환합니다.

    # table_number에 따라 table_name을 설정합니다.
    table_names = {1: '테이블1', 2: '테이블2', 3: '테이블3'}
    table_name = table_names.get(table_number, '테이블1')  # 기본값은 '테이블1'

    app = QApplication(sys.argv)
    gui = RestaurantRobotGUI(node, table_number, table_name)
    gui.show()

    # ROS2 스피닝을 별도의 스레드에서 시작합니다.
    ros_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    rclpy.shutdown()
    ros_thread.join()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
