#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import os

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        # Variables to manage subprocesses
        self.gazebo_process = None
        self.nav_process = None

        # Create a publisher for /initialpose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Start the launch files
        self.start_launches()

        # Create a timer to publish the initial pose after 5 seconds
        self.timer = self.create_timer(5.0, self.publish_initial_pose)
        self.timer_cancelled = False

    def start_launches(self):
        try:
            # 1. Start the turtlebot3_gazebo launch file
            gazebo_command = [
                'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'
            ]
            self.get_logger().info("Starting turtlebot3_gazebo launch...")
            self.gazebo_process = subprocess.Popen(gazebo_command)
            self.get_logger().info("turtlebot3_gazebo launch started successfully.")

            # 2. Start the navigation launch file
            map_path = os.path.expandvars(os.path.expanduser('$HOME/map.yaml'))
            if not os.path.isfile(map_path):
                self.get_logger().error(f"Map file not found at: {map_path}")
                return

            nav_command = [
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                f'map:={map_path}'
            ]
            self.get_logger().info(f"Starting navigation launch with map: {map_path}")
            self.nav_process = subprocess.Popen(nav_command)
            self.get_logger().info("Navigation launch started successfully.")

        except Exception as e:
            self.get_logger().error(f"Exception occurred while launching: {e}")

    def publish_initial_pose(self):
        if self.timer_cancelled:
            return

        # Create the PoseWithCovarianceStamped message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'

        # Updated pose values
        initial_pose.pose.pose.position.x = -0.007121707778424025
        initial_pose.pose.pose.position.y = -0.017804037779569626
        initial_pose.pose.pose.position.z = 0.0

        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.004543583995733044
        initial_pose.pose.pose.orientation.w = 0.9999896778689636

        # Updated covariance values
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # Publish the initial pose
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info("Initial pose published successfully.")

        # Cancel the timer since we only need to publish once
        self.timer.cancel()
        self.timer_cancelled = True

    def destroy_node(self):
        # Clean up subprocesses if the node is destroyed
        if self.gazebo_process and self.gazebo_process.poll() is None:
            self.gazebo_process.terminate()
            self.get_logger().info("Terminated gazebo process.")
        if self.nav_process and self.nav_process.poll() is None:
            self.nav_process.terminate()
            self.get_logger().info("Terminated navigation process.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("InitialPoseSetter node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
