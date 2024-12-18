import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, TwistStamped
from linkattacher_msgs.srv import AttachLink, DetachLink

from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
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