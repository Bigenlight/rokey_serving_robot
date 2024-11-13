import sys
import signal
import sqlite3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QTableWidget, QTableWidgetItem
from PyQt5.QtCore import Qt, QTimer

class GuiNode(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt5 GUI with SQLite3')
        self.setGeometry(100, 100, 600, 400)

        # Set up the label
        self.label = QLabel('Waiting for messages...', self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 18px;")

        # Set up the table
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(['ID', 'Message'])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(True)

        # Set up the layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.table)

        # Set the central widget
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Initialize SQLite3 database
        self.init_db()

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.node = rclpy.create_node('pyqt5_gui_node')  # Consider renaming the node if desired
        self.subscription = self.node.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Set up a QTimer to periodically spin the ROS2 node
        self.timer = QTimer()
        self.timer.setInterval(100)  # 100 ms
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start()

        # Install signal handler for SIGINT
        signal.signal(signal.SIGINT, self.handle_sigint)

    def init_db(self):
        # Connect to the SQLite3 database (creates the file if it doesn't exist)
        self.conn = sqlite3.connect('customers.db')
        self.cursor = self.conn.cursor()

        # Create the messages table if it doesn't exist
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS messages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                content TEXT NOT NULL
            )
        ''')
        self.conn.commit()

        # Load existing messages into the table
        self.load_messages()

    def load_messages(self):
        self.cursor.execute('SELECT id, content FROM messages ORDER BY id ASC')
        rows = self.cursor.fetchall()
        self.table.setRowCount(len(rows))
        for row_idx, row_data in enumerate(rows):
            id_item = QTableWidgetItem(str(row_data[0]))
            id_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            content_item = QTableWidgetItem(row_data[1])
            content_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.table.setItem(row_idx, 0, id_item)
            self.table.setItem(row_idx, 1, content_item)

    def listener_callback(self, msg):
        # Update the label with the received message
        self.label.setText(f"Received: {msg.data}")

        # Insert the message into the database
        self.cursor.execute('INSERT INTO messages (content) VALUES (?)', (msg.data,))
        self.conn.commit()

        # Get the last inserted ID
        last_id = self.cursor.lastrowid

        # Add the new message to the table
        row_position = self.table.rowCount()
        self.table.insertRow(row_position)

        id_item = QTableWidgetItem(str(last_id))
        id_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
        content_item = QTableWidgetItem(msg.data)
        content_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        self.table.setItem(row_position, 0, id_item)
        self.table.setItem(row_position, 1, content_item)

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def handle_sigint(self, signum, frame):
        # Close the application gracefully on SIGINT
        self.close()

    def closeEvent(self, event):
        # Stop the timer to prevent spin_ros from being called after shutdown
        self.timer.stop()
        
        # Clean up ROS2 resources
        self.node.destroy_node()
        rclpy.shutdown()

        # Close the database connection
        self.cursor.close()
        self.conn.close()

        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    gui = GuiNode()
    gui.show()
    sys.exit(app.exec_())
