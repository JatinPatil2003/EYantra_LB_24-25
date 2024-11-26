from payload_service.srv import PayloadSW

class PayloadService():
    def __init__(self, node):
        self.node = node

        self.payload_service = self.node.create_client(PayloadSW, '/payload_sw')

        while not self.payload_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for payload_sw service...')

    def receive_payload(self):
        req = PayloadSW.Request()
        req.receive = True
        req.drop = False

        response = self.payload_service.call(req)
        if response is not None:
            return response
        else:
            self.node.get_logger().info('Payload_sw service call failed')
            return None
        
    def drop_payload(self):
        req = PayloadSW.Request()
        req.receive = False
        req.drop = True

        response = self.payload_service.call(req)
        if response is not None:
            return response
        else:
            self.node.get_logger().info('Payload_sw service call failed')
            return None