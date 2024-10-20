from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
import time
import math

import ur_move
import servoing
import tf_tools


def main(args=None):
    rclpy.init(args=args)

    node = Node("task2_node")
    ur_move_client = ur_move.ArmMovement()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.add_node(ur_move_client)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    servoing_client = servoing.Servoingervice(node)
    tf_utils = tf_tools.TF(node)

    if not input('Start Servoing: ') == "":
        servoing_client.activate_servoing()
        node.get_logger().info("Started Servoing")

    aruco_ids = servoing_client.aruco_ids
    aruco_ids.remove(12)
    node.get_logger().info(f'Aruco IDs are {aruco_ids}')

    initaial_rotate(node, servoing_client)
    node.get_logger().info("Initial Rotated")

    for id in aruco_ids:
        node.get_logger().info(f'Moving to ID: {id}')
        dir = go_to(f'obj_{id}', node, servoing_client, tf_utils)
        time.sleep(2)
        
        if dir > 0:
            node.get_logger().info("Magnet On")
            ur_move_client.magnet_on(f'box{id}')
            node.get_logger().info("Box was Left; Moving Right")
            time.sleep(2)
            node.get_logger().info("Moving to Drop")
            move_cmd_vel(0.5, -0.5, 0.2, node, servoing_client)
            time.sleep(1)
            node.get_logger().info("Magnet Off")
            ur_move_client.magnet_off(f'box{id}')

            time.sleep(0.5)
            node.get_logger().info("Box Remove")
            servoing_client.detach_box(f'box{id}')
            time.sleep(2)
        else:
            node.get_logger().info("Magnet On")
            ur_move_client.magnet_on(f'box{id}')
            node.get_logger().info("Box was Right; Moving Left")
            time.sleep(2)
            node.get_logger().info("Moving to Drop")
            move_cmd_vel(0.5, 0.5, 0.2, node, servoing_client)
            time.sleep(1)
            node.get_logger().info("Magnet Off")
            ur_move_client.magnet_off(f'box{id}')

            time.sleep(0.5)
            node.get_logger().info("Box Remove")
            servoing_client.detach_box(f'box{id}')
            time.sleep(2)


            
    while input() == "":
        break

    try:
        node.destroy_node()
        ur_move_client.destroy_node()
        rclpy.shutdown()
    except:
        pass

def move_cmd_vel(x, y, z, node, servoing_client):
    prevtime = time.time()

    while time.time() - prevtime < 3.0:
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.linear.y = y
        twist.twist.linear.z = z
        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)
    print('stop')

def initaial_rotate(node, servoing_client):
    prevtime = time.time()

    while time.time() - prevtime < 3.0:
        twist = TwistStamped()
        twist.twist.angular.y = math.pi / 2

        twist.header.stamp = node.get_clock().now().to_msg()
        servoing_client.twist_publisher.publish(twist)
    print('stop')

def go_to(box_name, node, servoing_client, tf_utils):
    transform = tf_utils.lookup("ee", box_name)
    translation = (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    )
    translation_ = translation
    node.get_logger().info("Moving EEF...")
    while not abs(translation[0]) < 0.05 and not abs(translation[1]) < 0.05:
        twist = TwistStamped()
        twist.twist.linear.x = translation_[0]
        twist.twist.linear.y = translation_[1]
        twist.twist.linear.z = translation_[2]

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
    return translation_[1]

if __name__ == "__main__":
    main()
