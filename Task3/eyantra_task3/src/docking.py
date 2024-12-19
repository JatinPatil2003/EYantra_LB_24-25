from ebot_docking.srv import DockSw
from geometry_msgs.msg import Twist
import time

class DockService():
    def __init__(self, node):
        self.node = node

        self.dock_service = self.node.create_client(DockSw, '/dock_control')

        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        while not self.dock_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for dock_sw service...')

    def dock(self, bool_ori, yaw=0.0):
        req = DockSw.Request()
        req.linear_dock = True
        req.orientation_dock = bool_ori
        req.orientation = yaw

        response = self.dock_service.call(req)
        if response is not None:
            return response
        else:
            self.node.get_logger().info('dock_sw service call failed')
            return None
        
    def undock(self):
        prevtime = time.time()
        while time.time() - prevtime < 1.0:
            twist = Twist()
            twist.linear.x = 1.0
            self.twist_pub.publish(twist)
        self.twist_pub.publish(Twist())