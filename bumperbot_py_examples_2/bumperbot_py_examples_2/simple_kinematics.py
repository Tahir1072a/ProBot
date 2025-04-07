import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.get_transform_srv = self.create_service(GetTransform, "get_transform", self.get_transform_callback)
    
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

    def get_transform_callback(self, req, res):
        self.get_logger().info(f"Requested Transform between {req.frame_id} and {req.child_frame_id}")
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time()) # 0 yazsakda son zamanı alır.
        except TransformException as e:
            self.get_logger().error(f"An error occured while transfroming {req.frame_id} and {req.child_frame_id}")
            res.success = False
        
        res.transform = requested_transform
        res.success = True
        return res

def main():
    rclpy.init()

    node = SimpleTfKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()