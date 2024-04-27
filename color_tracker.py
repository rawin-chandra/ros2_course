#By Teacher S, S Adademy

#! /usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import time

class Robot(Node):

    def __init__(self):
        super().__init__('color_track')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            '/color',   #image_raw
            self.color_callback,
            10
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dir = -1    #-1 stop , 0,1,2,3  -> F,B,L,R
        self.timer_set = False
        self.count = 0
        self.deaf = False


    def forward(self):
        print("go forward")
        vel = Twist()
        vel.linear.x = 0.4
        self.vel_pub.publish(vel)


    def arc_left(self):
        print("arc left")
        vel = Twist()
        vel.linear.x = 0.4
        vel.angular.z = 1.2
        self.vel_pub.publish(vel)


    def arc_right(self):
        print("arc right")
        vel = Twist()
        vel.linear.x = 0.4
        vel.angular.z = -1.2
        self.vel_pub.publish(vel)


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

    def timer_callback(self):
        if self.dir == -1:
            self.stop()
        elif self.dir == 0:
            self.forward()
        elif self.dir == 2:
            self.arc_left()
        elif self.dir == 3:
            self.arc_right()

        if self.timer_set == True:
            self.count += 1

        if self.count > 30:   #3 seconds
            self.count = 0
            if self.deaf == True:
                self.deaf = False
                self.timer_set = False


    def color_callback(self, msg):
        if self.deaf == True:
            return

        if msg.data == "go":
            self.dir = 0
            self.deaf = True
            self.timer_set = True
        elif msg.data == "left":
            self.dir = 2
            self.deaf = True
            self.timer_set = True
        elif msg.data == "right":
            self.dir = 3
            self.deaf = True
            self.timer_set = True
        elif msg.data == "stop":
            self.dir = -1
            self.deaf = True
            self.timer_set = True


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
