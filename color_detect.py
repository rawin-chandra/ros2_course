#By S Academy

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String

class YellowBallFollower(Node):
    def __init__(self):
        super().__init__('red_ball_follower')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_pub = self.create_publisher(String, 'servo', 10)

        self.subscription = self.create_subscription(
            Image,
            '/image',   #image_raw
            self.image_callback,
            qos_profile_sensor_data  #10
        )
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.count = 0


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

    def image_callback(self, msg):
        self.count += 1
        #if self.count % 3 != 0:
        #    return

        move = False
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        #lower_yellow = np.array([20, 100, 100])
        #upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours are found
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(largest_contour)

            #if area < 500:   #800
            #    print("STOP too small")
            #    self.stop()
            #    return

            print("AREA = " , area)


            if area > 50000:  #2500   4000   2500
                print("REACH and GRAP!!!")
                self.stop()
                msg = String()
                msg.data = "grap"
                self.servo_pub.publish(msg)
                return

            # Get the centroid of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw a circle at the centroid of the largest contour
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                print(" ", cx, " ", cy)

                vel = Twist()

                if cx < 70:
                    print("LEFT")
                    move = True
                    vel.angular.z = 1.2
                    self.vel_pub.publish(vel)
                    #msg.data = "left"
                    #self.color_pub.publish(msg)
                elif cx > 200:  #250:
                    print("RIGHT")
                    move = True
                    vel.angular.z = -1.2
                    self.vel_pub.publish(vel)
                    #msg.data = "right"
                    #self.color_pub.publish(msg)
                else:
                    print("GO!!!")
                    vel.linear.x = 0.4
                    self.vel_pub.publish(vel)
                    #msg.data = "go"
                    #self.color_pub.publish(msg)
                    move = True

        if move == False:
            self.stop()
            #msg.data = "stop"
            #self.color_pub.publish(msg)
            print("STOP")

        cv2.imshow('yellow Ball Follower', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    yellow_ball_follower = YellowBallFollower()

    rclpy.spin(yellow_ball_follower)

    yellow_ball_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
