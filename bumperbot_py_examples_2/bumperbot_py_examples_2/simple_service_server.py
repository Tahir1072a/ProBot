import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.service = self.create_service(AddTwoInts, "add_two_ints", self.service_callback)
        self.get_logger().info("Service is ready")

    def service_callback(self, req, res):
        self.get_logger().info(f"New request received: {req.a} + {req.b}")
        
        res.sum = req.a + req.b
        self.get_logger().info(f"Result: {res.sum}")

        return res
    
def main():
    rclpy.init()
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()