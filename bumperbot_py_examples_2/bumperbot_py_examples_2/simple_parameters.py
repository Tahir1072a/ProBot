import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameters")

        self.declare_parameter("age", 22)
        self.declare_parameter("name", "Tahiri")

        self.add_on_set_parameters_callback(self.parameter_callback)

    # Runtime'da değişiklik yapmayı sağlar
    def parameter_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "age" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Age changed from {self.get_parameter('age').value} to {param.value}")
                result.successful = True
            
            if param.name == "name" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Name changed from {self.get_parameter('name').value} to {param.value}")
                result.successful = True

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()