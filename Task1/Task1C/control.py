#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
import tf2_ros
import numpy as np


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Transform listener to get the current transform between base_link and end_effector
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for periodic control
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Goal position and orientation (you can change these to desired values)
        self.goal_position = np.array([0.0, 0.0, 0.65])  # Example goal in meters
        self.goal_orientation = [0.0, 0.0, 0.0]  # Example orientation (Euler angles)

        self.linear_threshold = 0.05  # Threshold for linear distance to the goal
        self.angular_threshold = 0.1  # Threshold for angular distance to the goal

    def control_loop(self):
        try:
            # Get the transform from base_link to end_effector
            transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())

            # Extract translation and rotation (quaternion)
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert translation to a NumPy array
            current_position = np.array([trans.x, trans.y, trans.z])

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            current_orientation = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            # Compute the positional error between current position and goal position
            position_error = self.goal_position - current_position

            # Compute the angular error between current orientation and goal orientation
            angular_error = np.array(self.goal_orientation) - np.array(current_orientation)

            # Calculate linear and angular velocities based on errors (simple proportional control)
            linear_velocity = position_error * 5.0  # Scale by a constant factor
            angular_velocity = angular_error * 5.0  # Scale by a constant factor

            # Check if the robot is within the threshold to stop
            # if np.linalg.norm(position_error) < self.linear_threshold:
            #     linear_velocity = np.zeros(3)
            # if np.linalg.norm(angular_error) < self.angular_threshold:
            #     angular_velocity = np.zeros(3)

            # Create a TwistStamped message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = linear_velocity[0]
            twist_msg.twist.linear.y = linear_velocity[1]
            twist_msg.twist.linear.z = linear_velocity[2]
            twist_msg.twist.angular.x = angular_velocity[0]
            twist_msg.twist.angular.y = angular_velocity[1]
            twist_msg.twist.angular.z = angular_velocity[2]

            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist_msg)

        except tf2_ros.LookupException as e:
            self.get_logger().warn('Transform not found: ' + str(e))
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn('Extrapolation error: ' + str(e))


def main(args=None):
    rclpy.init(args=args)

    arm_controller = ArmController()

    rclpy.spin(arm_controller)

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
