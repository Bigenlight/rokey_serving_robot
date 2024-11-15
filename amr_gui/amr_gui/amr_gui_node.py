import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus  # GoalStatus 임포트

class AMRGui(QWidget):
    # PyQt5 신호 정의 (클래스 수준)
    show_standby_signal = pyqtSignal()
    show_status_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node  # AMRGuiNode 참조
        self.initUI()

        # 신호와 슬롯 연결
        self.show_standby_signal.connect(self.show_standby)
        self.show_status_signal.connect(self.show_status)

    @pyqtSlot()
    def show_standby(self):
        print("show_standby_signal received in show_standby.")
        self.status_label.hide()
        self.gif_label.show()
        self.gif_label.raise_()  # gif_label을 최상위 레이어로 올림
        if self.movie.state() != QMovie.Running:
            self.movie.start()

    @pyqtSlot(str)
    def show_status(self, status):
        print(f"show_status_signal received with status: {status}")
        self.gif_label.hide()
        self.status_label.setText(status)
        self.status_label.show()
        self.status_label.raise_()  # status_label을 최상위 레이어로 올림
        if self.movie.state() == QMovie.Running:
            self.movie.stop()

    def initUI(self):
        self.setWindowTitle('AMR 식당 로봇 GUI')
        self.setGeometry(100, 100, 700, 500)  # 창 크기 설정

        # GIF 레이블
        self.gif_label = QLabel(self)
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.gif_label.setScaledContents(True)

        # ROS2 패키지 내 이미지 경로
        package_share = get_package_share_directory('amr_gui')
        gif_path = os.path.join(package_share, 'images', 'eye-6662_256.gif')

        # QMovie 객체
        self.movie = QMovie(gif_path)
        if not self.movie.isValid():
            print(f"GIF 파일을 불러올 수 없습니다: {gif_path}")
        else:
            self.gif_label.setMovie(self.movie)
            self.movie.start()

        # 상태 메시지 레이블
        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        # 스타일 시트 수정: 글씨 굵게 및 검은색으로 변경
        self.status_label.setStyleSheet("font-size: 48px; color: black; font-weight: bold;")
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.hide()  # 초기에는 숨김

        # 긴급 정지 버튼
        self.emergency_button = QPushButton('긴급 정지', self)
        self.emergency_button.setStyleSheet("background-color: red; color: white; font-size: 20px;")
        # 정수 변환 적용
        self.emergency_button.setGeometry(int(self.width() - 110), int(self.height() - 80), 100, 60)
        self.emergency_button.clicked.connect(self.emergency_stop)

        # 주방 복귀 버튼
        self.return_button = QPushButton('주방 복귀', self)
        self.return_button.setStyleSheet("background-color: green; color: white; font-size: 20px;")
        # 정수 변환 적용
        self.return_button.setGeometry(10, int(self.height() - 80), 100, 60)
        self.return_button.clicked.connect(self.return_to_kitchen)

        # 음식 배달 시작 버튼 제거

        # 창 크기 변경 시 위젯 위치 조정
        self.resize_timer = QTimer()
        self.resize_timer.timeout.connect(self.resize_widgets)
        self.resize_timer.start(100)

    def resize_widgets(self):
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.emergency_button.setGeometry(int(self.width() - 110), int(self.height() - 80), 100, 60)
        self.return_button.setGeometry(10, int(self.height() - 80), 100, 60)
        # 음식 배달 시작 버튼 위치 조정 부분 제거

    def emergency_stop(self):
        print("긴급 정지 버튼이 눌렸습니다!")
        # 긴급 정지 메시지 발행 (Bool 타입)
        self.emergency_pub.publish(Bool(data=True))
        # 노드의 cancel_navigation 메서드 호출
        self.node.cancel_navigation()
        # GUI에 상태 메시지 표시 (신호 발행)
        self.show_status_signal.emit("긴급 정지!")

    def return_to_kitchen(self):
        print("주방 복귀 버튼이 눌렸습니다!")
        # 노드의 navigate_to_kitchen 메서드 호출
        self.node.navigate_to_kitchen()
        # GUI에 상태 메시지 표시 (신호 발행)
        self.show_status_signal.emit("주방 복귀중입니다...")

class AMRGuiNode(Node):
    def __init__(self):
        super().__init__('amr_gui_node')
        self.get_logger().info('AMRGuiNode initialized.')

        # 긴급 정지 퍼블리셔 (Bool 타입)
        self.publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.get_logger().info('Publisher for /emergency_stop created.')

        # 긴급 정지 구독
        self.emergency_subscription = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10)
        self.get_logger().info('Subscription to /emergency_stop created.')

        # AMR 상태 구독
        self.subscription = self.create_subscription(
            String,
            'amr_status',
            self.status_callback,
            10)
        self.get_logger().info('Subscription to /amr_status created.')

        # NavigateToPose 액션 클라이언트 생성
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('ActionClient for NavigateToPose created.')

        # GUI 설정
        self.app = QApplication(sys.argv)
        self.gui = AMRGui(self)
        self.gui.show()

        # 퍼블리셔를 GUI에 연결
        self.gui.emergency_pub = self.publisher

        # ROS2와 GUI 통합을 위한 타이머
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.update_gui)
        self.get_logger().info('AMRGuiNode setup complete.')

    def status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f'Received status: {status}')
        if status == "standby":
            self.gui.show_standby_signal.emit()
        elif status == "emergency_stop":
            self.gui.show_status_signal.emit("긴급 정지!")
        elif status == "return_to_kitchen":
            self.gui.show_status_signal.emit("주방 복귀중입니다...")
        elif status == "delivering":
            self.gui.show_status_signal.emit("음식 배달 중입니다...")
        else:
            self.gui.show_status_signal.emit(status)

    def update_gui(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.app.processEvents()
        self.get_logger().debug('update_gui called.')

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('긴급 정지 신호 수신 (Bool: True).')
            self.cancel_navigation()
            self.gui.show_status_signal.emit("긴급 정지!")
        else:
            self.get_logger().info('긴급 정지 해제 신호 수신 (Bool: False).')
            self.gui.show_standby_signal.emit()

    def cancel_navigation(self):
        """
        모든 NavigateToPose 액션 목표를 취소합니다.
        """
        self.get_logger().info('Navigation 취소 요청 전송 중...')
        # CancelGoal 서비스 클라이언트 생성
        cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if not cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose cancel_goal 서비스에 연결할 수 없습니다!')
            return

        # 모든 목표를 취소하기 위해 빈 CancelGoal 요청 생성
        request = CancelGoal.Request()

        future = cancel_client.call_async(request)
        future.add_done_callback(self.cancel_navigation_callback)

    def cancel_navigation_callback(self, future):
        """
        Navigation 취소 요청 후 콜백.
        """
        try:
            response = future.result()
            if len(response.goals_canceling) > 0:
                self.get_logger().info('Navigation 목표가 성공적으로 취소되었습니다.')
                self.gui.show_standby_signal.emit()
            else:
                self.get_logger().warn('취소할 Navigation 목표가 없습니다.')
        except Exception as e:
            self.get_logger().error(f'Navigation 취소 중 오류 발생: {e}')

    def navigate_to_kitchen(self):
        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose 액션 서버가 이용 가능하지 않습니다!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = -0.007121707778424025
        goal_msg.pose.pose.position.y = -0.017804037779569626
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.004543583995733044
        goal_msg.pose.pose.orientation.w = 0.9999896778689636

        self.get_logger().info('주방으로 이동 목표 전송 중...')
        self.gui.show_status_signal.emit("주방 복귀중입니다...")  # GUI에 상태 메시지 표시

        print("Sending goal to action server...")  # 디버깅 출력
        self._send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigate_feedback_callback)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, "주방 복귀"))

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Navigation 피드백 수신 중...')
        # 추가적인 피드백 처리가 필요하면 여기에 구현

    def goal_response_callback(self, future, action_name):
        print(f"goal_response_callback called for {action_name}.")  # 추가적인 콘솔 출력
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'{action_name} 목표 전송 실패: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().info(f'{action_name} 목표가 거부되었습니다.')
            self.gui.show_standby_signal.emit()  # 목표 거부 시 이미지로 복귀
            return

        self.get_logger().info(f'{action_name} 목표가 수락되었습니다.')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, action_name))

    def get_result_callback(self, future, action_name):
        print(f"get_result_callback called for {action_name}.")  # 콜백 호출 확인
        try:
            result_response = future.result()
            print(f"Result structure: {result_response}")  # 결과 구조 출력

            # status와 result 추출
            status = result_response.status
            result = result_response.result

            print(f"Status: {status}")
            print(f"Result: {result}")

            if status == GoalStatus.STATUS_SUCCEEDED:
                print(f"{action_name} 성공적으로 완료됨.")
            else:
                print(f"{action_name} 실패 또는 취소됨. Status: {status}")

            # 신호 발행 전 디버깅 출력
            print("Emitting show_standby_signal")
            self.gui.show_standby_signal.emit()
        except Exception as e:
            self.get_logger().error(f'{action_name} 완료 처리 중 오류 발생: {e}')
            print(f"{action_name} 완료 처리 중 오류 발생: {e}")  # 추가적인 콘솔 출력

    def destroy_node(self):
        super().destroy_node()
        self.app.quit()  # 애플리케이션이 제대로 종료되도록

def main(args=None):
    rclpy.init(args=args)
    amr_gui_node = AMRGuiNode()
    try:
        amr_gui_node.app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        amr_gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
