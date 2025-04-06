import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Joint tanımlamaları ile yapılan transform dönüşümünü manuel olarak nasıl yapıldığını gösteren bir örnek
class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_transform_stamped = TransformStamped()
        self.dynamic_transform_stamped = TransformStamped()

        self.increment_x = 0.05
        self.last_x = 0.0

        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = "transform_base"
        self.static_transform_stamped.child_frame_id = "transform_top"
        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.3
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(self.static_transform_stamped)

        self.get_logger().info(f"Publishin static transform between {self.static_transform_stamped.header.frame_id} and {self.static_transform_stamped.child_frame_id}")

        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        self.dynamic_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "transform_base"
        self.dynamic_transform_stamped.transform.translation.x = self.last_x + self.increment_x
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.x = 0.0
        self.dynamic_transform_stamped.transform.rotation.y = 0.0
        self.dynamic_transform_stamped.transform.rotation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.w = 1.0
    
        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped)
        self.last_x = self.dynamic_transform_stamped.transform.translation.x + self.increment_x

def main():
    rclpy.init()

    node = SimpleTfKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()