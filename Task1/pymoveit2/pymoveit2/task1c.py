#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, TwistStamped
from linkattacher_msgs.srv import AttachLink, DetachLink

from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread

from moveit2 import MoveIt2
from robots import ur5
import time


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
            execute_via_moveit=False,
        )

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.gripper_control_attach = self.create_client(AttachLink, '/GripperMagnetON')

        self.gripper_control_detach = self.create_client(DetachLink, '/GripperMagnetOFF')


    def move_arm_sequence(self, position, quat, is_cartsian):
        # Define the positions
        self.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat)}}}"
        )

        self.moveit2.move_to_pose(position=position, quat_xyzw=quat, cartesian=is_cartsian)
        self.moveit2.wait_until_executed()
    
    def move_arm_angles(self, joint_angles):
        self.get_logger().info(f"Moving to {{joint_positions: {joint_angles}}}")
        self.moveit2.move_to_configuration(joint_angles)
        self.moveit2.wait_until_executed()

    def move_servo(self, vel):
        curr_time = time.time()
        while(time.time() - curr_time <= 3.0):
            __twist_msg = TwistStamped()
            __twist_msg.header.stamp = self.get_clock().now().to_msg()
            __twist_msg.header.frame_id = ""
            __twist_msg.twist.linear.x = vel[0]
            __twist_msg.twist.linear.y = vel[1]
            __twist_msg.twist.linear.z = vel[2]
            __twist_msg.twist.angular.x = vel[3]
            __twist_msg.twist.angular.y = vel[4]
            __twist_msg.twist.angular.z = vel[5]
            self.cmd_vel_publisher.publish(__twist_msg)

    def magnet_on(self, box_name):
        req = AttachLink.Request()
        req.model1_name =  box_name  
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_attach.call_async(req)

    def magnet_off(self, box_name):
        req = DetachLink.Request()
        req.model1_name =  box_name  
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_detach.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ArmMovement()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    A = [[0.20, -0.45, 0.65], [0.707, 0.0, 0.0, 0.707]]
    B = [[0.75, 0.49, 0.0], [-0.707, 0.707, 0.0, 0.0]]
    C = [[0.75, -0.23, 0.0], [-0.707, 0.707, 0.0, 0.0]]
    D = [[-0.69, 0.10, 0.44], [0.0, -0.707, 0.0, 0.707]]

    home_angles = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
    A_init_angles = [-1.57, -2.39, 2.4, -3.15, -1.58, 3.15]
    A_angle = [-1.31, -1.52, 1.41, -3.04, -1.85, 3.14]
    # A = [-75, -87, 81, -174, -106, 180]
    # D_angle = [0.0, -2.02, -1.4, 0.28, 1.57, 0.0]
    # D = [0.0, -116, -80, 16, 90, 0]
    D_1_angle = [0.0, -1.83, -1.68, 0.33, 1.57, 0.0]
    D_2_angle = [0.0, -2.5, -0.61, -0.05, 1.57, 0.0]
    # D_1 = [0.0, -105, -96, 19, 90, 0]
    # D_2 = [0.0, -143, -35, -3, 90, 0]
    BC_init_angles = [0.0, -1.78, 1.45, -1.26, -1.57, 3.14]
    # BC = [0.0, -102, 83, -72, -90, 180]
    B_angle = [-0.45, -0.49, 1.15, -2.22, -1.57, 2.69]
    # B = [-26, -28, 66, -127, -90, 154]
    C_angle = [0.37, -0.31, 0.75, -2.01, -1.57, -2.69]
    # C = [21, -18, 43, -115, -90, -160]

    
    # -A= box1, B= box49, C = box3

    try:
        # A Box Pick n Place
        #####################################################################
        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        # node.move_arm_angles(A_init_angles)
        # node.get_logger().info("At A_init")
        node.move_arm_angles(A_angle)
        node.get_logger().info("At A")
        # time.sleep(1)
        node.magnet_on("box1")
        # time.sleep(0.5)

        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        node.move_arm_angles(D_1_angle)
        node.get_logger().info("At D")
        # time.sleep(1)
        node.magnet_off("box1")
        # time.sleep(0.5)

        # B Box Pick n Place
        #####################################################################

        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        # node.move_arm_angles(BC_init_angles)
        # node.get_logger().info("At BC_init")
        node.move_arm_angles(B_angle)
        node.get_logger().info("At B")
        # time.sleep(1)
        node.magnet_on("box49")
        # time.sleep(0.5)

        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        node.move_arm_angles(D_2_angle)
        node.get_logger().info("At D")
        # time.sleep(1)
        node.magnet_off("box49")
        # time.sleep(0.5)


        # C Box Pick n Place
        #####################################################################

        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        # node.move_arm_angles(BC_init_angles)
        # node.get_logger().info("At BC_init")
        node.move_arm_angles(C_angle)
        node.get_logger().info("At C")
        # time.sleep(1)
        node.magnet_on("box3")
        # time.sleep(0.5)

        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)

        node.move_arm_angles(D_2_angle)
        node.get_logger().info("At D")
        # time.sleep(1)
        node.magnet_off("box3")
        # time.sleep(0.5)

        

        

        # Finally Go Home
        #####################################################################
        # node.move_arm_angles(home_angles)
        # node.get_logger().info("At Home")
        # time.sleep(2)


        

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()



"""
[0.20, -0.0, 0.65]
[0.707, 0.0, 0.0, 0.707]

[0.20, -0.47, 0.65]
[0.707, 0.0, 0.0, 0.707]


ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "header: auto
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.2
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" -r 125


"""