import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import math

class TF:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_broadcast = TransformBroadcaster(self.node)

        self.timer_tf_broadcast = node.create_timer(0.05, self.tf_publisher)

    def tf_publisher(self):
        transform = self.lookup('base_link', 'ee_link')
        transform.child_frame_id = 'ee'
        transform.header.frame_id = 'base_link'
        quat = transform.transform.rotation
        euler = list(tf.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)))
        eular = [0.0, 0.0, 0.0]
        euler[0] = 0.0
        euler[1] = 0.0
        euler[2] = 0.0
        quat = tf.quaternion_from_euler(*tuple(euler))
        # print(euler)

        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcast.sendTransform(transform=transform)
        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.25
        transform.child_frame_id = 'drop'
        self.tf_broadcast.sendTransform(transform=transform)


    def lookup(self, to_frame, from_frame):
        try:
            transform = self.tf_buffer.lookup_transform_full(
                to_frame, rclpy.time.Time(), from_frame, rclpy.time.Time(), 'world', timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return transform
            
        except Exception as e:
            self.node.get_logger().error(f"TF lookup error: {e}")
            return None


def transformStampedToMatrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = np.eye(4)
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    rotation_matrix = tf.quaternion_matrix([
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w
    ])
    matrix[:3, :3] = rotation_matrix[:3, :3]
    return matrix

def matrixToTransform(matrix):
    transform = TransformStamped()
    transform.transform.translation.x = matrix[0, 3]
    transform.transform.translation.y = matrix[1, 3]
    transform.transform.translation.z = matrix[2, 3]
    quaternion = tf.quaternion_from_matrix(matrix)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform