#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion

import payload
import docking

import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
import time


def main(args=None):
    rclpy.init(args=args)

    node = Node('task2_node')
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    navigator = BasicNavigator()
    payload_client = payload.PayloadService(node)
    docking_client = docking.DockService(node)

    

    # drop_pose = get_pose(navigator, 0.49, -2.45, 0.0)
    drop_pose = get_pose(navigator, 0.6, -2.4, 3.14)

    # conveyor_1_pose = get_pose(navigator, -4.6, 3.1, 2.0)
    conveyor_2_pose = get_pose(navigator, 2.3, 3.1, 1.57)

    starttime = time.time()
    print(f'Start Time: {time.time()}')

    node.get_logger().info("Go to Drop Pose")
    go_to_pose(navigator, drop_pose)
    
    docking_client.dock(False)
    print(payload_client.receive_payload())

    node.get_logger().info("Go to Conveyor 2")
    go_to_pose(navigator, conveyor_2_pose)
    docking_client.dock(True, -1.57)
    print(payload_client.drop_payload())
    time.sleep(2)
    print(payload_client.drop_payload())


    print(f'End Time: {time.time()}')
    endtime = time.time()

    print(f'Total Time: {endtime - starttime} seconds')

    # print(payload_client.receive_payload())
    # time.sleep(3)
    # print(payload_client.drop_payload())

    # node.get_logger().info("Dockin Started")

    # docking_client.dock()


    navigator.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

def get_pose(navigator, x: float, y: float, theta: float):
    q = tf_transformations.quaternion_from_euler(0, 0, theta)
    q = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation = q
    return goal_pose

def go_to_pose(navigator, pose: PoseStamped):
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Task completed successfully!')
        return result

if __name__ == '__main__':
    main()