#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.get_logger().info('Simple Controller Node has been started.')

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        #self.get_parameter("wheel_radius", self.wheel_radius)
        #self.get_parameter("wheel_separation", self.wheel_separation)
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f'Wheel radius: {self.wheel_radius}')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}')

        self.prev_pos_right_wheel = 0.0
        self.prev_pos_left_wheel = 0.0
        self.prev_time = self.get_clock().now()

        self.wheel_command_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.cmd_vel_callback, 10)
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)

        self.speed_conversion = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                                          [self.wheel_radius / self.wheel_separation, -self.wheel_radius / self.wheel_separation]])
        
        self.get_logger().info(f'Speed conversion matrix: {self.speed_conversion}')

    def cmd_vel_callback(self, msg):
        #self.get_logger().info(f'cmd_vel: {msg}')
        
        robot_speed = np.array([[msg.twist.linear.x],[msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        self.get_logger().info(f'Wheel speed: {wheel_speed}')
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_command_pub.publish(wheel_speed_msg)

    def joint_callback(self, msg):
        dp_left = msg.position[1] - self.prev_pos_left_wheel
        dp_right = msg.position[0] - self.prev_pos_right_wheel
        dt = Time.from_msg(msg.header.stamp) - self.prev_time

        self.prev_pos_left_wheel = msg.position[1]
        self.prev_pos_right_wheel = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear_vel = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2
        angular_vel = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation

        self.get_logger().info(f"Linear {linear_vel}, Angular {angular_vel}")


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()