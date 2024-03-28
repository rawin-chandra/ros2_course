'''
code for LED testing in Ninja Turtle Robot of Teacher S

'''

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

import pigpio
import time

from rclpy.node import Node


class Led(Node):

    def __init__(self):
        super().__init__('led_test')
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.pi = pigpio.pi()

        self.pi.set_mode(2, pigpio.OUTPUT)
        self.pi.set_mode(3, pigpio.OUTPUT)
        self.pi.set_mode(4, pigpio.OUTPUT)
        self.pi.set_mode(17, pigpio.OUTPUT)



    def timer_callback(self):
        pi.write(2,1)
        pi.write(3,1)
        pi.write(4,1)
        pi.write(17,1)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    led = Led()

    rclpy.spin(led)

    led.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
