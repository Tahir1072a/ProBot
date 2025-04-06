import rclpy
from rclpy.node import Node

from bumperbot_msgs.srv import AddTwoInts

import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = AddTwoInts.Request()
        self.req.a = int(a)
        self.req.b = int(b)

        self.promise = self.client.call_async(self.req) # response bekleniyor...
        self.promise.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        self.get_logger().info(f"Response: {future.result().sum}")


def main(args=None):
    rclpy.init()

    if len(sys.argv) != 3:
        print("Wrong number of arguments")
        return -1

    node = SimpleServiceClient(sys.argv[1], sys.argv[2])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()