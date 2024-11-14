#amr_gui_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import Qt, QTimer
from ament_index_python.packages import get_package_share_directory  # ROS2 패키지 경로 가져오기
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal


class AMRGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # Reference to AMRGuiNode
        self.initUI()

    def initUI(self):
        self.setWindowTitle('AMR 식당 로봇 GUI')
        self.setGeometry(100, 100, 700, 500)  # 창 크기 설정

        # GIF 표시를 위한 QLabel
        self.gif_label = QLabel(self)
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.gif_label.setScaledContents(True)

        # ROS2 패키지 내에서 이미지 경로 설정
        package_share = get_package_share_directory('amr_gui')
        gif_path = os.path.join(package_share, 'images', 'eye-6662_256.gif')

        # QMovie 객체 생성
        self.movie = QMovie(gif_path)
        if not self.movie.isValid():
            print(f"GIF 파일을 로드할 수 없습니다: {gif_path}")
        else:
            self.gif_label.setMovie(self.movie)
            self.movie.start()

        # 상태 메시지 표시를 위한 QLabel
        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 48px; color: white;")
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.hide()  # 초기에는 숨김

        # 긴급 정지 버튼
        self.emergency_button = QPushButton('긴급 정지', self)
        self.emergency_button.setStyleSheet("background-color: red; color: white; font-size: 20px;")
        self.emergency_button.setGeometry(self.width() - 50, self.height() - 120, 100, 60)
        self.emergency_button.clicked.connect(self.emergency_stop)

        # 창 크기 변경 시 위젯 위치 및 크기 조정
        self.resize_timer = QTimer()
        self.resize_timer.timeout.connect(self.resize_widgets)
        self.resize_timer.start(100)

    def resize_widgets(self):
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.emergency_button.move(self.width() - 110, self.height() - 80)

    def show_standby(self):
        self.status_label.hide()
        self.gif_label.show()
        if self.movie.state() != QMovie.Running:
            self.movie.start()

    def show_status(self, status):
        self.gif_label.hide()
        self.status_label.setText(status)
        self.status_label.show()

    def emergency_stop(self):
        print("긴급 정지 버튼이 눌렸습니다!")
        # 긴급 정지 토픽에 메시지 퍼블리시
        self.emergency_pub.publish(String(data="emergency_stop"))
        # Call the node's cancel_navigation method
        self.node.cancel_navigation()


class AMRGuiNode(Node):
    def __init__(self):
        super().__init__('amr_gui_node')
        self.subscription = self.create_subscription(
            String,
            'amr_status',
            self.status_callback,
            10)
        self.publisher = self.create_publisher(String, 'emergency_stop', 10)

        # GUI setup
        self.app = QApplication(sys.argv)
        self.gui = AMRGui(self)
        self.gui.show()

        # Connect publisher to GUI
        self.gui.emergency_pub = self.publisher

        # Timer to integrate ROS2 and GUI
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.update_gui)

    def status_callback(self, msg):
        status = msg.data
        if status == "standby":
            self.gui.show_standby()
        else:
            self.gui.show_status(status)

    def update_gui(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.app.processEvents()

    def cancel_navigation(self):
        """
        Cancel all NavigateToPose action goals.
        """
        self.get_logger().info('Sending navigation cancel request...')
        # Create a client for the cancel goal service
        cancel_client = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        if not cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Unable to connect to NavigateToPose cancel goal service!')
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
