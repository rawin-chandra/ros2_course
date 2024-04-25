#! /usr/bin/env python3
#by Teacher S

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import time

class Robot(Node):

    def __init__(self):
        super().__init__('robot_move')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)


    def move(self):
        print("forward")
        vel = Twist()
        vel.linear.x = 0.4
        self.vel_pub.publish(vel)
        time.sleep(5)
        self.stop()
        time.sleep(2)
        print("backward")
        vel.linear.x = -0.4
        self.vel_pub.publish(vel)
        time.sleep(5)
        self.stop()
        time.sleep(2)
        print("left")
        vel.linear.x = 0.0
        vel.angular.z = 1.2
        self.vel_pub.publish(vel)
        time.sleep(5)
        self.stop()
        time.sleep(2)
        print("right")
        vel.linear.x = 0.0
        vel.angular.z = -1.2
        self.vel_pub.publish(vel)
        time.sleep(5)
        self.stop()


    def stop(self):
        print("############### STOP ##################")
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    robot.move()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
