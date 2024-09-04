#!/usr/bin/env python3

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import tf_transformations


def quaternion_from_yaw(yaw):
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class EBotNavigator(Node):

    def __init__(self):
        super().__init__('ebot_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.orientation = quaternion_from_yaw(pose[2])

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Sending goal: [{pose[0]}, {pose[1]}, {pose[2]}]')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')

    def navigate_through_poses(self, poses):
        for pose in poses:
            self.get_logger().info(f'Navigating to pose: {pose}')
            goal_future = self.send_goal(pose)

            # Wait for the action to complete
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().info('Goal was rejected')
                return

            self.get_logger().info('Goal accepted, waiting for result')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            # self.get_logger().info(f'{result}')
            if result.status == 4:
                self.get_logger().info(f'Successfully reached: {pose}')
            else:
                self.get_logger().info('Failed to reach the goal')
                return
            
            time.sleep(2)

        self.get_logger().info('Navigation through all poses completed')


def main(args=None):
    rclpy.init(args=args)

    navigator = EBotNavigator()

    poses = [
        [-0.12, -2.35, 3.14],
        [1.86, 2.56, 0.97],
        [-3.84, 2.64, 2.78]
    ]

    navigator.navigate_through_poses(poses)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
