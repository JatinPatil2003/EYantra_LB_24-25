#!/usr/bin/env python3

from example_interfaces.srv import SetBool

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
import time
import math
import subprocess
import signal

import ur_move
import servoing
import tf_tools



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetBool, 'passing_service', self.passing_service_callback)

        self.node = Node("task3b_node")
        self.ur_move_client = ur_move.ArmMovement()

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self.node)
        executor.add_node(self.ur_move_client)
        # executor.add_node(minimal_service)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        self.servoing_client = servoing.Servoingervice(self.node)
        self.tf_utils = tf_tools.TF(self.node)

        # if not input('Start Servoing: ') == "":
        self.servoing_client.activate_servoing()
        self.node.get_logger().info("Started Servoing")
        initaial_rotate(self.node, self.servoing_client)
        self.node.get_logger().info("Initial Rotated")  

    def passing_service_callback(self, request, response):
        def update_box_list(done_list, servoing_client):
            aruco_list =  servoing_client.aruco_ids
            for i in done_list:
                try:
                    aruco_list.remove(i)
                except:
                    pass
            # aruco_list.sort()
            return aruco_list


        done_list = [12]
        aruco_ids = update_box_list(done_list, self.servoing_client)
        self.node.get_logger().info(f'Aruco IDs are {aruco_ids}')

        id = aruco_ids[0]
        self.node.get_logger().info(f'Moving to ID: {id}')
        dir = go_to(f'obj_{id}', self.node, self.servoing_client, self.tf_utils)

        self.node.get_logger().info("Magnet On")
        self.ur_move_client.magnet_on(f'box{id}')
        self.node.get_logger().info("Moving to Drop")
        # time.sleep(2)
        # go_to(f'drop', self.node, self.servoing_client, self.tf_utils)
        # # time.sleep(2)
        # # move_cmd_vel(0.0, 0.0, -0.15, self.node, self.servoing_client)
        # self.node.get_logger().info("Magnet Off")
        # self.ur_move_client.magnet_off(f'box{id}')
        # time.sleep(0.3)
        
        response.success = True

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()
    

    
       
    rclpy.spin(minimal_service)
    rclpy.shutdown()

def move_cmd_vel(x, y, z, node, servoing_client):
    prevtime = time.time()

    while time.time() - prevtime < 0.9:
        twist = TwistStamped()
        twist.twist.linear.x = x * 4.0
        twist.twist.linear.y = y * 4.0
        twist.twist.linear.z = z * 4.0
        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)
    print('stop')

def initaial_rotate(node, servoing_client):
    prevtime = time.time()

    while time.time() - prevtime < 2.7:
        twist = TwistStamped()
        twist.twist.linear.x = 0.2
        twist.twist.angular.y = math.pi / 2

        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)

def go_to(box_name, node, servoing_client, tf_utils):
    transform = tf_utils.lookup("ee", box_name)
    translation = (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z + 0.04,
    )
    translation_ = translation
    node.get_logger().info("Moving EEF...")
    prevtime = time.time()
    while not abs(translation[0]) < 0.03 and not abs(translation[1]) < 0.03 and time.time() - prevtime < 2.0:
    # while not abs(translation[0]) < 0.05 and not abs(translation[1]) < 0.05:
        twist = TwistStamped()
        twist.twist.linear.x = translation_[0] * 2.0
        twist.twist.linear.y = translation_[1] * 2.0
        twist.twist.linear.z = translation_[2] * 2.0

        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)

        try:
            transform = tf_utils.lookup("ee", box_name)
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
        except Exception as e:
            print(e)
            break
    # servoing_client.twist_publisher.publish(TwistStamped())
    node.get_logger().info("Stopping EEF...")
    servoing_client.twist_publisher.publish(TwistStamped())
    return translation_[1]


if __name__ == '__main__':
    main()