import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sys
import os
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient


class Communicator(QObject):
    # Define signals to communicate between threads
    update_status = pyqtSignal(str)
    show_standby = pyqtSignal()
    show_status = pyqtSignal(str)  # Renamed from show_status_signal to show_status


class AMRGui(QWidget):
    def __init__(self, communicator):
        super().__init__()
        self.communicator = communicator  # Communicator for inter-thread signals
        self.initUI()
        self.connect_signals()

    def initUI(self):
        self.setWindowTitle('AMR 식당 로봇 GUI')
        self.setGeometry(100, 100, 700, 500)  # Window size

        # GIF Label
        self.gif_label = QLabel(self)
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.gif_label.setScaledContents(True)

        # Image path within ROS2 package
        package_share = get_package_share_directory('amr_gui')
        gif_path = os.path.join(package_share, 'images', 'eye-6662_256.gif')

        # QMovie object
        self.movie = QMovie(gif_path)
        if not self.movie.isValid():
            print(f"Cannot load GIF file: {gif_path}")
        else:
            self.gif_label.setMovie(self.movie)
            self.movie.start()

        # Status message label
        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 48px; color: white;")
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.hide()  # Initially hidden

        # Emergency Stop Button
        self.emergency_button = QPushButton('긴급 정지', self)
        self.emergency_button.setStyleSheet("background-color: red; color: white; font-size: 20px;")
        self.emergency_button.setGeometry(self.width() - 110, self.height() - 80, 100, 60)  # Adjusted position
        self.emergency_button.clicked.connect(self.emergency_stop)

        # Return to Kitchen Button
        self.return_button = QPushButton('주방 복귀', self)
        self.return_button.setStyleSheet("background-color: green; color: white; font-size: 20px;")
        self.return_button.setGeometry(10, self.height() - 80, 100, 60)
        self.return_button.clicked.connect(self.return_to_kitchen)

        # Adjust widgets on window resize
        self.resize_timer = QTimer()
        self.resize_timer.timeout.connect(self.resize_widgets)
        self.resize_timer.start(100)

    def connect_signals(self):
        # Connect communicator signals to GUI slots
        self.communicator.show_standby.connect(self.show_standby)
        self.communicator.show_status.connect(self.show_status)  # Now correctly connects to show_status

    def resize_widgets(self):
        self.gif_label.setGeometry(0, 0, self.width(), self.height())
        self.status_label.setGeometry(0, 0, self.width(), self.height())
        self.emergency_button.move(self.width() - 110, self.height() - 80)
        self.return_button.move(10, self.height() - 80)

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
        # Emit signal to ROS node to publish emergency stop
        self.communicator.update_status.emit("emergency_stop")
        # Alternatively, use a direct method call if thread-safe

    def return_to_kitchen(self):
        print("주방 복귀 버튼이 눌렸습니다!")
        # Emit signal to ROS node to navigate to kitchen
        self.communicator.update_status.emit("navigate_to_kitchen")
        # Alternatively, use a direct method call if thread-safe


class AMRGuiNode(Node):
    def __init__(self, communicator):
        super().__init__('amr_gui_node')
        self.get_logger().info('AMRGuiNode initialized.')

        self.communicator = communicator  # Communicator for inter-thread signals

        # Publisher for emergency stop (Bool type)
        self.publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.get_logger().info('Publisher for /emergency_stop created.')

        # Subscription to emergency stop topic
        self.emergency_subscription = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10)
        self.get_logger().info('Subscription to /emergency_stop created.')

        # Subscription to amr_status
        self.subscription = self.create_subscription(
            String,
            'amr_status',
            self.status_callback,
            10)
        self.get_logger().info('Subscription to /amr_status created.')

        # Create NavigateToPose action client
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('ActionClient for NavigateToPose created.')

        # Connect communicator signal to node slots
        self.communicator.update_status.connect(self.handle_gui_commands)

        self._goal_handle = None  # Initialize goal handle

    def status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f'Received status: {status}')
        if status == "standby":
            self.communicator.show_standby.emit()
        else:
            self.communicator.show_status.emit(status)  # Correctly emits show_status

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received emergency stop signal (Bool: True).')
            self.cancel_navigation()
        else:
            self.get_logger().info('Received emergency stop signal with Bool: False.')

    def handle_gui_commands(self, command):
        if command == "emergency_stop":
            self.publish_emergency_stop()
            self.cancel_navigation()
        elif command == "navigate_to_kitchen":
            self.navigate_to_kitchen()

    def publish_emergency_stop(self):
        msg = Bool(data=True)
        self.publisher.publish(msg)
        self.get_logger().info('Published emergency stop signal.')

    def cancel_navigation(self):
        """
        Cancel all NavigateToPose action goals.
        """
        self.get_logger().info('Sending navigation cancel request...')
        # Correct service name for the CancelGoal service
        cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
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


    def navigate_to_kitchen(self):
        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
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

        self.get_logger().info('Sending goal to return to kitchen.')

        self._send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigate_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback.')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation to kitchen completed.')
        # Handle the result as needed

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info('AMRGuiNode destroyed.')


def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    communicator = Communicator()

    # Initialize ROS node
    amr_gui_node = AMRGuiNode(communicator)

    # Start ROS node in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(amr_gui_node,), daemon=True)
    ros_thread.start()

    # Initialize and run GUI in the main thread
    app = QApplication(sys.argv)
    gui = AMRGui(communicator)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        amr_gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
