#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose

from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


class ArmMovement(Node):
    def __init__(self):
        super().__init__("arm_movement")
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

    def move_arm_sequence(self, position, quat, is_cartsian):
        # Define the positions
        positions = [
            [0.20, -0.47, 0.65],
            [-0.69, 0.10, 0.44],
            [0.75, 0.49, -0.05],
            [-0.69, 0.10, 0.44],
            [0.75, -0.23, -0.05],
            [-0.69, 0.10, 0.44],
        ]

        self.moveit2.move_to_pose(position=position, quat_xyzw=quat, cartesian=is_cartsian)
        self.moveit2.wait_until_executed()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMovement()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    positions = [
            [0.20, -0.47, 0.65],
            [-0.69, 0.10, 0.44],
            [0.75, 0.49, -0.05],
            [-0.69, 0.10, 0.44],
            [0.75, -0.23, -0.05],
            [-0.69, 0.10, 0.44],
        ]

    while True:
        node.get_logger().info("running")


    rclpy.shutdown()


if __name__ == "__main__":
    main()
