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
        
        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.last_angular_z = 0.0 #odometry noisy topic'inden gelen değeri depolamak için.

        self.motion = 0.0
        self.kalman_odom = Odometry()

    def imu_callback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z

    def odom_callback(self, odom):
        self.kalman_odom = odom

        if self.is_first_odom:
            self.mean = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z

            self.is_first_odom = False
            return

        self.statePrediction() # implemente edilecek

        self.mesaurementUpdate() # implemente edilecek
    