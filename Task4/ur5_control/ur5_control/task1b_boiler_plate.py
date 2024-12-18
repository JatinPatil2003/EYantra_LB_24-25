#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf_transformations as tf

from std_msgs.msg import String


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    area = None
    width = None

    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = coordinates
    
    area = 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (x2*y1 + x3*y2 + x4*y3 + x1*y4))

    return area, width


def detect_aruco(image, depth_image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    def get_distance(depth_image, x, y):
        if depth_image is None:
            return 0.0
        return depth_image[y, x] / 1000.0

    aruco_area_threshold = 1500

    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    # # Detect the markers
    # corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_m, cam_mat, dist_mat)

        for i in range(len(ids)):
            coordinates = corners[i][0]

            area, width = calculate_rectangle_area(coordinates)

            if area < aruco_area_threshold:
                continue

            cX = int(np.mean(coordinates[:, 0]))
            cY = int(np.mean(coordinates[:, 1]))
            center_aruco_list.append((cX, cY))

            distance_from_rgb_list.append(get_distance(depth_image, cX, cY))

            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            # euler = r.as_euler('xyz', degrees=False)
            # # euler[0] += np.pi
            # # euler[1] += np.pi
            # # euler[2] += np.pi
            # rotated_euler = R.from_euler('xyz', euler)
            quat = r.as_quat()

            angle_aruco_list.append(quat)

            width_aruco_list.append(width)

            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvecs[i], tvecs[i], 0.1)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

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

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.aruco_string_pub = self.create_publisher(String, '/detected_aruco', 10)
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None 
        self.mat_base_to_laser = None                                                        # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def get_base_to_camera_tf(self):
        if self.mat_base_to_laser is not None:
            return 
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', rclpy.time.Time())
            self.mat_base_to_laser = transformStampedToMatrix(transform)

        except Exception as ex:
            self.get_logger().error('Transform lookup failed: %s' % str(ex))

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        self.get_base_to_camera_tf()

        if self.cv_image is None or self.depth_image is None:
            return

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image, self.depth_image)

        if ids is None or self.mat_base_to_laser is None:
            return
        
        self.aruco_string_pub.publish(String(data=str(ids)))
            
        for i, marker_id in enumerate(ids):
            try:
                cX, cY = center_aruco_list[i]
                distance = distance_from_rgb_list[i]
                quat = angle_aruco_list[i]

                x = distance * (cX - centerCamX) / focalX
                y = distance * (cY - centerCamY) / focalY
                z = distance

                transform = TransformStamped()
                transform.transform.translation.x = x
                transform.transform.translation.y = y
                transform.transform.translation.z = z
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]

                self.publish_transform(transform, marker_id[0])
            except:
                pass

        # Optionally, display the image with markers drawn (for debugging)
        cv2.imshow('Aruco Detection', self.cv_image)
        cv2.waitKey(1)

    def publish_transform(self, transform, id):

        mat_camera_to_box_tf = transformStampedToMatrix(transform)
        
        mat_base_to_box_tf = tf.concatenate_matrices(
            self.mat_base_to_laser,
            mat_camera_to_box_tf
        )

        t = matrixToTransform(mat_base_to_box_tf)
        quat = t.transform.rotation
        euler = list(tf.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)))
        euler[0] += math.pi
        euler[2] += math.pi
        quat = tf.quaternion_from_euler(*tuple(euler))

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'obj_{id}'

        self.br.sendTransform(t)


def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
