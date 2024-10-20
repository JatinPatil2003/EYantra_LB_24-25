from servo_msgs.srv import ServoLink
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class Servoingervice():
    def __init__(self, node):
        self.node = node

        self.servo_service = self.node.create_client(ServoLink, '/SERVOLINK')
        
        self.servoing_activator = self.node.create_client(Trigger, '/servo_node/start_servo')

        self.twist_publisher = self.node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.detected_aruco_sub = self.node.create_subscription(String, '/detected_aruco', self.aruco_callback, 10)

        while not self.servo_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for servolink service service...')

        while not self.servoing_activator.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for servoing_activator service service...')

    def aruco_callback(self, msg):
        self.aruco_ids = []
        arr = msg.data[1:-1].split('\n')
        for id  in arr:
            id = int(id.strip().strip('[]').strip())
            if id not in self.aruco_ids:
                self.aruco_ids.append(id)

    def activate_servoing(self):
        req = Trigger.Request()
        self.servoing_activator.call_async(req)

    def detach_box(self, box_name):
        req = ServoLink.Request()
        req.box_name = box_name
        req.box_link = 'link'

        response = self.servo_service.call(req)
        if response is not None:
            return response
        else:
            self.node.get_logger().info('Payload_sw service call failed')
            return None
