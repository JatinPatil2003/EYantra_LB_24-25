#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from moveit_msgs.action import MoveGroup
# from moveit_msgs.msg import RobotState
# from geometry_msgs.msg import Pose

# from rclpy.callback_groups import ReentrantCallbackGroup
# from threading import Thread

# from moveit2 import MoveIt2
# from robots import ur5
# import time


# class ArmMovement(Node):
#     def __init__(self):
#         super().__init__("arm_movement")
#         self.callback_group = ReentrantCallbackGroup()
#         self.moveit2 = MoveIt2(
#             node=self,
#             joint_names=ur5.joint_names(),
#             base_link_name=ur5.base_link_name(),
#             end_effector_name=ur5.end_effector_name(),
#             group_name=ur5.MOVE_GROUP_ARM,
#             callback_group=self.callback_group,
#         )

#     def move_arm_sequence(self, position, quat, is_cartsian):
#         # Define the positions
#         self.get_logger().info(
#             f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat)}}}"
#         )

#         self.moveit2.move_to_pose(position=position, quat_xyzw=quat, cartesian=is_cartsian)
#         self.moveit2.wait_until_executed()


# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmMovement()

#     executor = rclpy.executors.MultiThreadedExecutor(2)
#     executor.add_node(node)
#     executor_thread = Thread(target=executor.spin, daemon=True, args=())
#     executor_thread.start()

#     try:
#         positions = [
#             [0.20, -0.47, 0.65],
#             [-0.69, 0.10, 0.44],
#             [0.75, 0.49, -0.05],
#             [-0.69, 0.10, 0.44],
#             [0.75, -0.23, -0.05],
#             [-0.69, 0.10, 0.44],
#         ]
#         quaternion = [
#             [0.707, 0.0, 0.0, 0.707],
#             [0.0, 0.707, 0.0, 0.707],
#             [-0.707, 0.707, 0.0, 0.0],
#             [0.0, 0.707, 0.0, 0.707],
#             [-0.707, 0.707, 0.0, 0.0],
#             [0.0, 0.707, 0.0, 0.707],
#         ]  # Replace with actual quaternion values if needed
#         is_cartesian = False  # Set according to your needs

#         for i in range(len(positions)):
#             node.move_arm_sequence(positions[i], quaternion[i], is_cartesian)
#             time.sleep(3)

#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down")
#     finally:
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from moveit2 import MoveIt2
from robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [-0.69, 0.10, 0.44])
    node.declare_parameter("quat_xyzw", [0.0, -0.707, 0.0, 0.707])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

