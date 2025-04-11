#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub = self.create_subscription(Odometry,"/bumperbot_controller/odom_noisy", self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/out", self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/bumperbot_controller/kalman_odom",10)

        self.mean = 0.0
        self.variance = 1000.0
        
        self.imu_angular_z = 0.0 # Imu'dan gelen son angular_z mesajını depolar
        self.is_first_odom = True
        self.last_angular_z = 0.0 # odometry noisy topic'inden gelen değeri depolamak için.

        self.motion = 0.0 # Ardışık zamanda gelen iki angular_velocity değerinin farkını depolayacaktır.
        self.kalman_odom = Odometry()

        self.motion_variance = 4.0
        self.measurement_variance = 0.5

    def measurement_update(self):
        self.mean = (self.measurement_variance * self.mean + self.variance * self.imu_angular_z) / (self.variance + self.measurement_variance)
        self.variance = (self.variance * self.measurement_variance) / (self.variance + self.measurement_variance)
    
    # Bir sonraki adımı tahmin eder. Yani bir sonraki adımda oluşacak olan gauss dağılımını hesaplarız.
    def state_prediction(self):
        self.mean = self.mean + self.motion
        self.variance = self.variance + self.motion_variance

    def imu_callback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z

    def odom_callback(self, odom):
        self.kalman_odom = odom

        if self.is_first_odom:
            self.last_angular_z = odom.twist.twist.angular.z
            self.mean = odom.twist.twist.angular.z

            self.is_first_odom = False
            return

        self.motion = odom.twist.twist.angular.z - self.last_angular_z

        self.state_prediction() 
        self.measurement_update() 

        self.last_angular_z = odom.twist.twist.angular.z

        self.kalman_odom.twist.twist.angular.z = self.mean
        self.odom_pub.publish(self.kalman_odom)

def main():
    rclpy.init()
    node = KalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    