#! /usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg   import LaserScan

class Laser(Node):

    def __init__(self):
        super().__init__('laser_test')
        self.laser_sub = self.create_subscription(LaserScan,'scan', self.laser_callback,qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.laser_sub

    def laser_callback(self, msg):
        print(" size = " , len(msg.ranges))
        print(" angle = " , msg.angle_increment)

        for i in range(0,len(msg.ranges),2):
            print(" " , i , " " , msg.ranges[i])


def main(args=None):
    rclpy.init(args=args)
    laser = Laser()
    rclpy.spin(laser)

    laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
