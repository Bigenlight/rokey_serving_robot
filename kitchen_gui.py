import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGroupBox, QButtonGroup
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class KitchenGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("주방 GUI")
        self.setGeometry(100, 100, 1000, 600)
        self.initUI()

    def initUI(self):
        # 메인 레이아웃을 세 개의 칼럼으로 분리
        main_layout = QHBoxLayout(self)

        # 왼쪽 칼럼: 주문 내역과 주문 내역 표시 영역
        left_column = QVBoxLayout()
        order_label = QLabel("주문 내역")
        order_label.setAlignment(Qt.AlignCenter)
        order_label.setFixedHeight(30)  # 글씨 부분 높이 줄이기
        left_column.addWidget(order_label)

        # 각 테이블의 주문 내역 표시 영역
        for table in ["테이블A", "테이블B", "테이블C"]:
            table_layout = QVBoxLayout()
            table_name = QLabel(table)
            table_name.setAlignment(Qt.AlignCenter)
            table_name.setFixedHeight(20)  # 글씨 부분 높이 줄이기
            table_layout.addWidget(table_name)

            order_details = QLabel("주문 내역 표시 영역")
            order_details.setStyleSheet("background-color: #003366; color: white;")
            order_details.setFixedHeight(150)  # 주문 내역 표시 영역 높이 늘리기
            table_layout.addWidget(order_details)

            left_column.addLayout(table_layout)

        main_layout.addLayout(left_column, 1)

        # 중앙 칼럼: DB 버튼과 테이블별 버튼들
        center_column = QVBoxLayout()
        db_button = QPushButton("DB 확인")
        db_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
        center_column.addWidget(db_button)

        # 각 테이블별 버튼 그룹
        for table in ["테이블A", "테이블B", "테이블C"]:
            table_group = QGroupBox(table + " 버튼")
            table_layout = QVBoxLayout()

            order_reject_button = QPushButton("주문 거절")
            order_reject_button.setEnabled(False)  # 기본 비활성화
            order_reject_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            order_reject_button.clicked.connect(lambda _, tb=table: self.reject_order(tb))
            table_layout.addWidget(order_reject_button)

            out_of_stock_button = QPushButton("재료 부족")
            out_of_stock_button.setEnabled(False)
            out_of_stock_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            out_of_stock_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "재료 부족"))
            table_layout.addWidget(out_of_stock_button)

            not_in_mood_button = QPushButton("하기 싫음")
            not_in_mood_button.setEnabled(False)
            not_in_mood_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            not_in_mood_button.clicked.connect(lambda _, tb=table: self.send_reject_reason(tb, "하기 싫음"))
            table_layout.addWidget(not_in_mood_button)

            cooking_done_button = QPushButton("조리 완료")
            cooking_done_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            cooking_done_button.clicked.connect(lambda _, tb=table: self.mark_cooking_done(tb))
            table_layout.addWidget(cooking_done_button)

            alarm_off_button = QPushButton("알람 끄기")
            alarm_off_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            alarm_off_button.setStyleSheet("background-color: grey;")
            alarm_off_button.clicked.connect(lambda: self.toggle_alarm_off(alarm_off_button))
            table_layout.addWidget(alarm_off_button)

            table_group.setLayout(table_layout)
            center_column.addWidget(table_group)

        main_layout.addLayout(center_column, 1)

        # 오른쪽 칼럼: 수동 조종 영역
        right_column = QVBoxLayout()
        control_label = QLabel("수동 조종")
        control_label.setAlignment(Qt.AlignCenter)
        control_label.setFixedHeight(30)  # 글씨 부분 높이 줄이기
        right_column.addWidget(control_label)

        # 테이블 선택 버튼 그룹
        table_selection = QGroupBox("테이블 선택")
        table_selection_layout = QVBoxLayout()

        self.table_button_group = QButtonGroup()
        self.table_button_group.setExclusive(False)

        for table in ["테이블 A", "테이블 B", "테이블 C"]:
            table_button = QPushButton(table)
            table_button.setCheckable(True)
            table_button.clicked.connect(self.handle_table_selection)
            self.table_button_group.addButton(table_button)
            table_selection_layout.addWidget(table_button)
        
        table_selection.setLayout(table_selection_layout)
        right_column.addWidget(table_selection)

        # 기능 버튼 영역
        function_group = QGroupBox("기능")
        function_layout = QHBoxLayout()
        for function in ["긴급 정지", "주방 복귀", "로봇 보내기"]:
            function_button = QPushButton(function)
            function_button.setFixedHeight(30)  # 글씨 부분 높이 줄이기
            function_layout.addWidget(function_button)
        function_group.setLayout(function_layout)
        right_column.addWidget(function_group)

        main_layout.addLayout(right_column, 1)

        self.setLayout(main_layout)

    def handle_table_selection(self):
        for button in self.table_button_group.buttons():
            if button.isChecked():
                button.setStyleSheet("background-color: yellow;")
            else:
                button.setStyleSheet("")

    def toggle_alarm_off(self, button):
        if button.styleSheet() == "background-color: red;":
            button.setStyleSheet("background-color: grey;")
        else:
            button.setStyleSheet("background-color: red;")

    def reject_order(self, table):
        print(f"{table}의 주문을 거절합니다.")

    def send_reject_reason(self, table, reason):
        print(f"{table}의 거절 이유: {reason}")

    def mark_cooking_done(self, table):
        print(f"{table}의 요리가 완료되었습니다.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KitchenGUI()
    window.show()
    sys.exit(app.exec_())
