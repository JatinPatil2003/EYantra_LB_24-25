#!/usr/bin/env python3

from example_interfaces.srv import SetBool
from payload_service.srv import PayloadSW


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from std_msgs.msg import String

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
        self.srv = self.create_service(PayloadSW, 'passing_service', self.passing_service_callback)

        self.node = Node("task3b_node")
        self.ur_move_client = ur_move.ArmMovement()

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self.node)
        executor.add_node(self.ur_move_client)
        # executor.add_node(minimal_service)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        self.timer = self.node.create_timer(1.0, self.timer_callback)

        self.go_drop = False

        self.aruco_ids = []
        self.servoing_client = servoing.Servoingervice(self.node)
        self.tf_utils = tf_tools.TF(self.node)
        self.detected_aruco_sub = self.create_subscription(String, '/detected_aruco', self.aruco_callback, 10)


        # if not input('Start Servoing: ') == "":
        self.servoing_client.activate_servoing()
        self.node.get_logger().info("Started Servoing")
        # initaial_rotate(self.node, self.servoing_client)
        # self.node.get_logger().info("Initial Rotated")  

        # self.ur_move_client.move_initial()
        initaial_rotate(self.node, self.servoing_client)

        # self.joint_names = ['shoulder_pan_joint']  # Name of the joint
        # self.displacements = [110 * 3.14159265359 / 180]

        # self.ur_move_client.move_joint_jog(self.joint_names, self.displacements)

    def timer_callback(self):
        # self.node.get_logger().info(f'{self.aruco_ids}')
        if self.go_drop:
            self.go_drop = False
            time.sleep(0.3)
            go_to(f'drop', self.node, self.servoing_client, self.tf_utils)

    def aruco_callback(self, msg):
        self.aruco_ids_new = []
        arr = msg.data[1:-1].split('\n')
        for id  in arr:
            id = int(id.strip().strip('[]').strip())
            if id not in self.aruco_ids_new:
                self.aruco_ids_new.append(id)
        if self.aruco_ids_new is not self.aruco_ids:
            self.aruco_ids = self.aruco_ids_new

    def passing_service_callback(self, request, response):
        def update_box_list(done_list, servoing_client):
            aruco_list =  self.aruco_ids
            for i in done_list:
                try:
                    aruco_list.remove(i)
                except:
                    pass
            aruco_list.sort()
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
        time.sleep(0.5)

        # self.ur_move_client.move_initial()
        go_to(f'drop', self.node, self.servoing_client, self.tf_utils)
        time.sleep(0.5)

        # time.sleep(2)
        go_to(f'drop_bot', self.node, self.servoing_client, self.tf_utils)
        # # move_cmd_vel(0.0, 0.0, -0.15, self.node, self.servoing_client)
        self.node.get_logger().info("Magnet Off")
        self.ur_move_client.magnet_off(f'box{id}')
        
        self.go_drop = True


        response.success = True
        response.message = f'{id}'

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
        transform.transform.translation.z + 0.03,
    )
    translation_ = translation
    node.get_logger().info("Moving EEF...")
    prevtime = time.time()
    while not abs(translation[0]) < 0.04 and not abs(translation[1]) < 0.04 and time.time() - prevtime < 2.0:
    # while not abs(translation[0]) < 0.05 and not abs(translation[1]) < 0.05:
        twist = TwistStamped()
        twist.twist.linear.x = translation_[0] * 1.5
        twist.twist.linear.y = translation_[1] * 1.5
        twist.twist.linear.z = translation_[2] * 1.5

        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)

        

        try:
            transform = tf_utils.lookup("ee", box_name)
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            if abs(translation[0]) < 0.04 or abs(translation[1]) < 0.04:
                node.get_logger().info("Limit Stop EEF...")
        except Exception as e:
            print(e)
            break
    # servoing_client.twist_publisher.publish(TwistStamped())
    node.get_logger().info("Stopping EEF...")
    servoing_client.twist_publisher.publish(TwistStamped())
    return translation_[1]


# def go_to(box_name, node, servoing_client, tf_utils):
#     transform = tf_utils.lookup("ee", box_name)
#     translation = (
#         transform.transform.translation.x,
#         transform.transform.translation.y,
#         transform.transform.translation.z + 0.03,
#     )
#     translation_ = translation
#     node.get_logger().info("Moving EEF...")
#     prevtime = time.time()
#     while not abs(translation[0]) < 0.04 and not abs(translation[1]) < 0.04 and time.time() - prevtime < 3.0:
#     # while not abs(translation[0]) < 0.05 and not abs(translation[1]) < 0.05:
#         twist = TwistStamped()
#         twist.twist.linear.x = translation_[0]
#         twist.twist.linear.y = translation_[1]
#         twist.twist.linear.z = translation_[2]

#         twist.header.stamp = node.get_clock().now().to_msg()
#         servoing_client.twist_publisher.publish(twist)

        

#         try:
#             transform = tf_utils.lookup("ee", box_name)
#             translation = (
#                 transform.transform.translation.x,
#                 transform.transform.translation.y,
#                 transform.transform.translation.z,
#             )
#             if abs(translation[0]) < 0.04 or abs(translation[1]) < 0.04:
#                 node.get_logger().info("Limit Stop EEF...")
#         except Exception as e:
#             print(e)
#             break
#     # servoing_client.twist_publisher.publish(TwistStamped())
#     node.get_logger().info("Stopping EEF...")
#     servoing_client.twist_publisher.publish(TwistStamped())
#     return translation_[1]

if __name__ == '__main__':
    main()