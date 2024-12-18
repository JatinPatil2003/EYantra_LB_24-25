#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics

import time

class DockingController(Node):

    def __init__(self):
        super().__init__('docking_server')

        self.callback_group = ReentrantCallbackGroup()

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)

        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)


        self.twist_publsiher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.is_docking = False
        self.dock_aligned = False
        self.robot_pose = [0.0, 0.0, 0.0]

        # self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw


    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    def imu_callback(self, msg):
        self.imu_value = msg

    def controller_loop(self):

        if self.is_docking:
            if self.angular_dock:
                prevtime = time.time()
                while time.time() - prevtime < 2.0 :
                    twist = Twist()
                    twist.angular.z = 1.57
                    self.twist_publsiher.publish(twist)
                self.twist_publsiher.publish(Twist())
                q = self.imu_value.orientation
                error = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - self.angular_orientation
                prevtime = time.time()
                print(error)
                while time.time() - prevtime < 1.0 and not abs(error) < 0.05:
                    twist = Twist()
                    twist.angular.z = -error 
                    self.twist_publsiher.publish(twist)
                self.get_logger().info("Orientation Dock Done")
                self.twist_publsiher.publish(Twist())
            if self.linear_dock:
                error = min(self.usrleft_value, self.usrright_value)
                prevtime = time.time()
                print(error)
                while time.time() - prevtime < 1.0 and not error < 0.1:
                    twist = Twist()
                    twist.linear.x = -max(error, 1.1)
                    self.twist_publsiher.publish(twist)
                self.get_logger().info("Linear Dock Done")
                self.twist_publsiher.publish(Twist())
            self.dock_aligned = True
            self.is_docking = False
            self.angular_dock = False
            self.linear_dock = False


    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.is_docking = True
        self.dock_aligned = False

        self.linear_dock = request.linear_dock
        self.angular_dock = request.orientation_dock
        self.angular_orientation = request.orientation

        self.get_logger().info("Docking started!")

        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned :
            self.get_logger().info("Waiting for alignment...")
            self.controller_loop()
            # rate.sleep()

        response.success = True
        response.message = "Docking control initiated"

        self.is_docking = False
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = DockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
