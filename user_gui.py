import sys
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QMessageBox, QStackedWidget, QMainWindow, QSpinBox, QScrollArea
)
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QObject, QEvent


class StaffCallThread(QThread):
    # 신호 정의: 호출 시작 및 완료
    call_started = pyqtSignal()
    call_completed = pyqtSignal()

    def run(self):
        self.call_started.emit()
        # 직원 호출 시뮬레이션 (예: 5초 대기)
        time.sleep(5)
        self.call_completed.emit()


class InactivityEventFilter(QObject):
    def __init__(self, reset_callback):
        super().__init__()
        self.reset_callback = reset_callback

    def eventFilter(self, obj, event):
        if event.type() in [QEvent.MouseMove, QEvent.KeyPress, QEvent.MouseButtonPress]:
            self.reset_callback()
        return False


class RestaurantRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("식당 서비스 로봇")
        self.setGeometry(100, 100, 400, 600)

        # 장바구니와 메뉴 데이터 초기화
        self.cart = {}
        self.menu = {
            "피자": ["pizza.png", 12000],
            "파스타": ["pasta.png", 10000],
            "샐러드": ["salad.png", 7000],
            "콜라": ["cola.png", 2000]
        }

        # 스택 위젯 생성
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # 각 화면 추가
        self.waiting_screen = self.create_waiting_screen()
        self.menu_screen = self.create_menu_screen()
        self.cart_screen = self.create_cart_screen()

        self.stack.addWidget(self.waiting_screen)
        self.stack.addWidget(self.menu_screen)
        self.stack.addWidget(self.cart_screen)

        # 직원 호출 스레드 초기화
        self.staff_call_thread = StaffCallThread()
        self.staff_call_thread.call_started.connect(self.on_staff_call_started)
        self.staff_call_thread.call_completed.connect(self.on_staff_call_completed)

        # 비활성화 타이머 초기화 (메뉴 화면용)
        self.inactivity_timer = QTimer()
        self.inactivity_timer.setInterval(5000)  # 5초
        self.inactivity_timer.setSingleShot(True)
        self.inactivity_timer.timeout.connect(self.return_to_waiting_screen)

        # 이벤트 필터 초기화 및 설치 (메뉴 화면)
        self.inactivity_event_filter = InactivityEventFilter(self.reset_inactivity_timer)
        self.menu_screen.installEventFilter(self.inactivity_event_filter)

        # 연결: 화면 변경 시 타이머 관리
        self.stack.currentChanged.connect(self.on_screen_changed)

    def create_waiting_screen(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        # 로고 이미지 추가
        logo_label = QLabel()
        logo_pixmap = QPixmap("logo.png").scaled(400, 300)
        logo_label.setPixmap(logo_pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)

        # 안내 텍스트
        label = QLabel("환영합니다!\n'주문하기'를 눌러주세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 16px;")
        layout.addWidget(label)

        # 직원 호출 상태 표시 라벨
        self.staff_call_status_label = QLabel("")
        self.staff_call_status_label.setAlignment(Qt.AlignCenter)
        self.staff_call_status_label.setStyleSheet("font-size: 14px; color: blue;")
        layout.addWidget(self.staff_call_status_label)

        # 주문하기 버튼
        order_button = QPushButton("주문하기")
        order_button.setFixedSize(200, 40)
        order_button.clicked.connect(self.show_menu_screen)
        layout.addWidget(order_button, alignment=Qt.AlignCenter)

        # 직원 호출 버튼
        call_staff_button = QPushButton("직원 호출")
        call_staff_button.setFixedSize(200, 40)
        call_staff_icon = QIcon(QPixmap("call_staff.png").scaled(40, 40))
        call_staff_button.setIcon(call_staff_icon)
        call_staff_button.setIconSize(QPixmap("call_staff.png").scaled(40, 40).size())
        call_staff_button.clicked.connect(self.call_staff)
        layout.addWidget(call_staff_button, alignment=Qt.AlignCenter)

        # 로봇 주방 복귀 버튼
        robot_return_button = QPushButton("로봇 주방 복귀")
        robot_return_button.setFixedSize(200, 40)
        robot_return_icon = QIcon(QPixmap("robot_return.png").scaled(40, 40))
        robot_return_button.setIcon(robot_return_icon)
        robot_return_button.setIconSize(QPixmap("robot_return.png").scaled(40, 40).size())
        robot_return_button.clicked.connect(self.return_robot)
        layout.addWidget(robot_return_button, alignment=Qt.AlignCenter)

        widget.setLayout(layout)
        return widget

    def create_menu_screen(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        label = QLabel("메뉴를 선택하세요")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 16px;")
        layout.addWidget(label)

        self.quantity_spinboxes = {}

        # 메뉴 항목 및 수량 선택 스핀박스 생성
        for item, (image_path, price) in self.menu.items():
            item_layout = QHBoxLayout()
            item_layout.setAlignment(Qt.AlignCenter)

            item_image = QLabel()
            item_pixmap = QPixmap(image_path).scaled(50, 50)
            item_image.setPixmap(item_pixmap)
            item_layout.addWidget(item_image)

            item_button = QPushButton(f"{item} - {price}원")
            item_button.clicked.connect(lambda _, i=item: self.add_to_cart(i))
            item_layout.addWidget(item_button)

            # 수량 선택 스핀박스
            spin_box = QSpinBox()
            spin_box.setRange(0, 10)
            item_layout.addWidget(spin_box)
            self.quantity_spinboxes[item] = spin_box

            layout.addLayout(item_layout)

        # 직원 호출 버튼 (추가)
        call_staff_button = QPushButton("직원 호출")
        call_staff_button.setFixedSize(200, 40)
        call_staff_icon = QIcon(QPixmap("call_staff.png").scaled(40, 40))
        call_staff_button.setIcon(call_staff_icon)
        call_staff_button.setIconSize(QPixmap("call_staff.png").scaled(40, 40).size())
        call_staff_button.clicked.connect(self.call_staff)
        layout.addWidget(call_staff_button, alignment=Qt.AlignCenter)

        # 장바구니 보기 버튼
        cart_button = QPushButton("장바구니 보기")
        cart_button.setFixedSize(200, 40)
        cart_button.clicked.connect(self.show_cart)
        layout.addWidget(cart_button, alignment=Qt.AlignCenter)

        # 초기 화면으로 돌아가는 버튼
        back_to_home_button = QPushButton("초기 화면으로 돌아가기")
        back_to_home_button.setFixedSize(200, 40)
        back_to_home_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.waiting_screen))
        layout.addWidget(back_to_home_button, alignment=Qt.AlignCenter)

        widget.setLayout(layout)
        return widget

    def create_cart_screen(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)

        label = QLabel("장바구니")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 16px;")
        layout.addWidget(label)

        # 장바구니 항목을 담을 스크롤 가능한 영역 추가
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        self.cart_items_layout = QVBoxLayout(scroll_content)
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        self.total_label = QLabel("총 금액: 0원")
        self.total_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.total_label)

        # 직원 호출 상태 표시 라벨
        self.cart_staff_call_status_label = QLabel("")
        self.cart_staff_call_status_label.setAlignment(Qt.AlignCenter)
        self.cart_staff_call_status_label.setStyleSheet("font-size: 14px; color: blue;")
        layout.addWidget(self.cart_staff_call_status_label)

        # 버튼 레이아웃
        buttons_layout = QHBoxLayout()

        pay_button = QPushButton("결제하기")
        pay_button.setFixedSize(150, 40)
        pay_button.clicked.connect(self.pay)
        buttons_layout.addWidget(pay_button)

        clear_button = QPushButton("장바구니 초기화")
        clear_button.setFixedSize(150, 40)
        clear_button.clicked.connect(self.clear_cart)
        buttons_layout.addWidget(clear_button)

        back_button = QPushButton("메뉴로 돌아가기")
        back_button.setFixedSize(200, 40)
        back_button.clicked.connect(lambda: self.stack.setCurrentWidget(self.menu_screen))
        buttons_layout.addWidget(back_button)

        # 직원 호출 버튼 (추가)
        cart_call_staff_button = QPushButton("직원 호출")
        cart_call_staff_button.setFixedSize(200, 40)
        cart_call_staff_icon = QIcon(QPixmap("call_staff.png").scaled(40, 40))
        cart_call_staff_button.setIcon(cart_call_staff_icon)
        cart_call_staff_button.setIconSize(QPixmap("call_staff.png").scaled(40, 40).size())
        cart_call_staff_button.clicked.connect(self.call_staff)
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
        QMessageBox.information(self, "시간 초과", "5초 동안 활동이 없어 대기 화면으로 돌아갑니다.")
        self.stack.setCurrentWidget(self.waiting_screen)

    def show_cart(self):
        self.update_cart_screen()
        self.stack.setCurrentWidget(self.cart_screen)

    def update_cart_screen(self):
        # 기존 레이아웃 항목 제거
        while self.cart_items_layout.count():
            item = self.cart_items_layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()

        # 장바구니 항목을 화면에 표시
        total = 0
        for item, quantity in self.cart.items():
            if quantity > 0:
                item_widget = QWidget()
                item_layout = QHBoxLayout()
                item_layout.setAlignment(Qt.AlignCenter)

                # 항목 라벨
                item_label = QLabel(f"{item} - {self.menu[item][1]}원 x {quantity}개")
                item_label.setAlignment(Qt.AlignCenter)
                item_layout.addWidget(item_label)

                # 수량 변경 스핀박스
                spin_box = QSpinBox()
                spin_box.setRange(1, 10)  # 최소 1개, 최대 10개
                spin_box.setValue(quantity)
                spin_box.valueChanged.connect(lambda val, i=item: self.change_quantity(i, val))
                item_layout.addWidget(spin_box)

                # 개별 삭제 버튼
                delete_button = QPushButton("삭제")
                delete_button.setFixedSize(60, 30)
                delete_button.clicked.connect(lambda _, i=item: self.remove_from_cart(i))
                item_layout.addWidget(delete_button)

                item_widget.setLayout(item_layout)
                self.cart_items_layout.addWidget(item_widget)

                total += self.menu[item][1] * quantity

        # 총 금액 표시
        self.total_label.setText(f"총 금액: {total}원")

    def clear_cart(self):
        # 장바구니 초기화
        self.cart.clear()
        QMessageBox.information(self, "장바구니", "장바구니가 초기화되었습니다.")
        self.update_cart_screen()

    def add_to_cart(self, item):
        # 선택된 수량을 장바구니에 추가
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
        # 장바구니 항목의 수량 변경
        if quantity > 0:
            self.cart[item] = quantity
        else:
            del self.cart[item]
        self.update_cart_screen()
        self.reset_inactivity_timer()

    def remove_from_cart(self, item):
        # 장바구니에서 특정 항목 삭제
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

        QMessageBox.information(self, "결제 완료", "결제가 완료되었습니다!")
        self.cart.clear()
        self.update_cart_screen()
        self.stack.setCurrentWidget(self.waiting_screen)

    def call_staff(self):
        # 직원 호출 스레드 시작
        if not self.staff_call_thread.isRunning():
            self.staff_call_thread.start()
        else:
            QMessageBox.warning(self, "직원 호출", "이미 직원 호출이 진행 중입니다.")

    def on_staff_call_started(self):
        # 호출 시작 시 상태 업데이트 및 버튼 비활성화
        current_widget = self.stack.currentWidget()
        if current_widget == self.waiting_screen:
            self.staff_call_status_label.setText("직원 호출 중...")
        elif current_widget == self.cart_screen:
            self.cart_staff_call_status_label.setText("직원 호출 중...")
        # 모든 화면의 직원 호출 버튼을 비활성화
        self.disable_staff_call_buttons()

    def on_staff_call_completed(self):
        # 호출 완료 시 상태 업데이트 및 버튼 활성화
        current_widget = self.stack.currentWidget()
        if current_widget == self.waiting_screen:
            self.staff_call_status_label.setText("직원이 도착했습니다!")
        elif current_widget == self.cart_screen:
            self.cart_staff_call_status_label.setText("직원이 도착했습니다!")
        # 모든 화면의 직원 호출 버튼을 활성화
        self.enable_staff_call_buttons()
        QMessageBox.information(self, "직원 호출", "직원이 도착했습니다!")

    def disable_staff_call_buttons(self):
        # 모든 직원 호출 버튼을 비활성화
        for screen in [self.waiting_screen, self.menu_screen, self.cart_screen]:
            call_buttons = screen.findChildren(QPushButton, "직원 호출")
            for button in call_buttons:
                button.setEnabled(False)

    def enable_staff_call_buttons(self):
        # 모든 직원 호출 버튼을 활성화
        for screen in [self.waiting_screen, self.menu_screen, self.cart_screen]:
            call_buttons = screen.findChildren(QPushButton, "직원 호출")
            for button in call_buttons:
                button.setEnabled(True)

    def return_robot(self):
        QMessageBox.information(self, "로봇 복귀", "로봇이 복귀합니다.")


    def closeEvent(self, event):
        # 애플리케이션 종료 시 스레드 종료 처리
        if self.staff_call_thread.isRunning():
            self.staff_call_thread.terminate()
            self.staff_call_thread.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RestaurantRobotGUI()
    window.show()
    sys.exit(app.exec_())
