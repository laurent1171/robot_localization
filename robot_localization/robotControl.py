import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import time
from numpy import inf
import pandas as pd
import numpy as np
from array import array

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.create_timer(0.1, self.run_loop)
        # self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # # self.odom_sub = self.create_subscription(Odometry, "odom", 10)
        # self.run_loop()


    def run_loop(self):
        pass


def main(args=None):
    pass

if __name__ == "__main__":
    main()
