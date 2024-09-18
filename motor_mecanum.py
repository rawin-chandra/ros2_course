import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pigpio
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class Motor(Node):

    def __init__(self):
        super().__init__('motor_control')
        self.sub_vel = self.create_subscription(Twist,'cmd_vel',self.vel_callback,10)
        self.robot_wheel_pub_left = self.create_publisher(String,'robot_wheel_vel_left',10)
        self.robot_wheel_pub_right = self.create_publisher(String,'robot_wheel_vel_right',10)

        self.subscription = self.create_subscription(
            Twist,
            'motor_vel_left',
            self.motor_callback_left,
            10)
        self.subscription = self.create_subscription(
            Twist,
            'motor_vel_right',
            self.motor_callback_right,
            10)
        self.subscription
        self.wheel_width = 0.28       #width of   0.26
        self.left_vel_up = 0
        self.left_vel_down = 0
        self.right_vel_up = 0
        self.right_vel_down = 0
        self.real_left_vel_up = 0
        self.real_left_vel_down = 0
        self.real_right_vel_up = 0
        self.real_right_vel_down = 0
        self.dir = 0    #0, 1 , 2 ,3 ,4 ,5 ,6 ,7 =>  fw, bw, L, R, LF, RF, LB, RB
                        #8, 9 => straf left, straf right

        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(3,100);  #left-up PWM
        self.pi.set_PWM_frequency(17,100);  #left-down PWM
        self.pi.set_PWM_frequency(22,100);  #right-up PWM
        self.pi.set_PWM_frequency(9,100);  #right-down PWM

        self.pi.set_mode(2, pigpio.OUTPUT) #left-up DIR
        self.pi.set_mode(4, pigpio.OUTPUT) #left-down DIR
        self.pi.set_mode(27, pigpio.OUTPUT) #right-up DIR
        self.pi.set_mode(10, pigpio.OUTPUT) #right-down DIR


        self.motor_val_left = [0,0]
        self.motor_val_right = [0,0]
        self.acc_param = 30  #3

    def motor_callback_left(self, msg):
        vals = [0,0,0,0]
        vals[0] = float(msg.linear.x)   #a_up
        vals[1] = float(msg.linear.y)   #a_down
        vals[2] = float(msg.angular.x)  #b_up
        vals[3] = float(msg.angular.y)  #b_down

        if self.dir == 0 or self.dir == 4 or self.dir == 5:           #**** edit from error in the book
           self.real_left_vel_up = vals[0]
           self.real_left_vel_down = vals[1]
        elif self.dir == 1 or self.dir == 6 or self.dir == 7:
           self.real_left_vel_up = -1 * vals[2]
           self.real_left_vel_down = -1 * vals[3]
        elif self.dir == 2:
           self.real_left_vel_up = -1 * vals[2]
           self.real_left_vel_down = -1 * vals[3]
        elif self.dir == 3:
           self.real_left_vel_up = vals[0]
           self.real_left_vel_down = vals[1]
        elif self.dir == 8:
           self.real_left_vel_up = -1 * vals[2]
           self.real_left_vel_down = vals[1]
        elif self.dir == 9:
           self.real_left_vel_up = vals[0]
           self.real_left_vel_down = -1 * vals[3]

        diff_left_up = abs(self.real_left_vel_up) - abs(self.left_vel_up)
        diff_left_down = abs(self.real_left_vel_down) - abs(self.left_vel_down)

        #left up
        self.motor_val_left[0] = self.motor_val_left[0] - (diff_left_up * self.acc_param)
        if self.motor_val_left[0] < 0:
           self.motor_val_left[0] = 0

        if self.left_vel_up == 0:
           self.motor_val_left[0] = 0

        self.motor_val_left[1] = self.motor_val_left[1] - (diff_left_down * self.acc_param)
        if self.motor_val_left[1] < 0:
           self.motor_val_left[1] = 0

        if self.left_vel_down == 0:
           self.motor_val_left[1] = 0

        if self.left_vel_up > 0:
           self.pi.write(2,1)
        else:
           self.pi.write(2,0)


        if self.left_vel_down > 0:
           self.pi.write(4,1)
        else:
           self.pi.write(4,0)

        val_up = min(self.motor_val_left[0],255)
        val_down = min(self.motor_val_left[1],255)

        self.pi.set_PWM_dutycycle(3,val_up)   #23
        self.pi.set_PWM_dutycycle(17,val_down)   #19

        msg = String()

        msg.data = '%f,%f' % (self.real_left_vel_up, self.real_left_vel_down)

        self.robot_wheel_pub_left.publish(msg)

        print("val up = ",val_up ," : val_down = ",val_down)


    def motor_callback_right(self, msg):
        vals = [0,0,0,0]
        vals[0] = float(msg.linear.x)   #a_up
        vals[1] = float(msg.linear.y)   #a_down
        vals[2] = float(msg.angular.x)  #b_up
        vals[3] = float(msg.angular.y)  #b_down

        if self.dir == 0 or self.dir == 4 or self.dir == 5:           #**** edit from error in the book
           self.real_right_vel_up = vals[2]
           self.real_right_vel_down = vals[3]
        elif self.dir == 1 or self.dir == 6 or self.dir == 7:
           self.real_right_vel_up = -1 * vals[0]
           self.real_right_vel_down = -1 * vals[1]
        elif self.dir == 2:
           self.real_right_vel_up = vals[2]
           self.real_right_vel_down = vals[3]
        elif self.dir == 3:
           self.real_right_vel_up = -1 * vals[0]
           self.real_right_vel_down = -1 * vals[1]
        elif self.dir == 8:
           self.real_right_vel_up = vals[2]
           self.real_right_vel_down = -1 * vals[1]
        elif self.dir == 9:
           self.real_right_vel_up = -1 * vals[0]
           self.real_right_vel_down = vals[3]

        diff_right_up = abs(self.real_right_vel_up) - abs(self.right_vel_up)
        diff_right_down = abs(self.real_right_vel_down) - abs(self.right_vel_down)

        #left up
        self.motor_val_right[0] = self.motor_val_right[0] - (diff_right_up * self.acc_param)
        if self.motor_val_right[0] < 0:
           self.motor_val_right[0] = 0

        if self.right_vel_up == 0:
           self.motor_val_right[0] = 0

        self.motor_val_right[1] = self.motor_val_right[1] - (diff_right_down * self.acc_param)
        if self.motor_val_right[1] < 0:
           self.motor_val_right[1] = 0

        if self.right_vel_down == 0:
           self.motor_val_right[1] = 0

        if self.right_vel_up > 0:
           self.pi.write(27,0)
        else:
           self.pi.write(27,1)


        if self.right_vel_down > 0:
           self.pi.write(10,0)
        else:
           self.pi.write(10,1)

        val_up = min(self.motor_val_right[0],255)
        val_down = min(self.motor_val_right[1],255)

        self.pi.set_PWM_dutycycle(22,val_up)   #23
        self.pi.set_PWM_dutycycle(9,val_down)   #19

        msg = String()

        msg.data = '%f,%f' % (self.real_right_vel_up, self.real_right_vel_down)

        self.robot_wheel_pub_right.publish(msg)



    def vel_callback(self, msg):
        #0, 1 , 2 ,3 ,4 ,5 ,6 ,7 =>  fw, bw, L, R, LF, RF, LB, RB
                        #8, 9 => straf left, straf right
        vel_x = msg.linear.x
        vel_y = msg.linear.y
        vel_th = msg.angular.z

        if vel_x == 0 and vel_y == 0 and vel_th == 0:
            self.left_vel_up = 0;
            self.left_vel_down = 0;
            self.right_vel_up = 0;
            self.right_vel_down = 0;
            return


        if vel_x == 0:
            if vel_y > 0:  #straffing left
               self.dir = 8
               self.left_vel_up = -1 * vel_y;
               self.left_vel_down = vel_y;
               self.right_vel_up = vel_y;
               self.right_vel_down = -1 * vel_y;
            elif vel_y < 0:
               self.dir = 9
               self.left_vel_up = -1 * vel_y;
               self.left_vel_down = vel_y;
               self.right_vel_up = vel_y;
               self.right_vel_down = -1 * vel_y;
            elif vel_th != 0: #rotate
               if vel_th > 0:
                   self.dir = 2   #left
                   self.left_vel_up = -1 * vel_th * self.wheel_width / 2.0;
                   self.left_vel_down = -1 * vel_th * self.wheel_width / 2.0;
                   self.right_vel_up = vel_th * self.wheel_width / 2.0;
                   self.right_vel_down = vel_th * self.wheel_width / 2.0;
               else:
                   self.dir = 3
                   self.left_vel_up = -1 * vel_th * self.wheel_width / 2.0;
                   self.left_vel_down = -1 * vel_th * self.wheel_width / 2.0;
                   self.right_vel_up = vel_th * self.wheel_width / 2.0;
                   self.right_vel_down = vel_th * self.wheel_width / 2.0;


        elif vel_th == 0:  #go straight
           if vel_x > 0:
               self.dir = 0
           else:
               self.dir = 1

           self.left_vel_up = vel_x;
           self.left_vel_down = vel_x;
           self.right_vel_up = vel_x;
           self.right_vel_down = vel_x;

        else:  #arc
           self.left_vel_up = vel_x - vel_th * self.wheel_width 
           self.left_vel_down = self.left_vel_up
           self.right_vel_up = vel_x + vel_th * self.wheel_width 
           self.right_vel_down = self.right_vel_up

           if self.left_vel_up > 0 and self.right_vel_up > 0:
              if self.left_vel_up == self.right_vel_up:
                 self.dir = 0
              elif self.left_vel_up > self.right_vel_up:
                 self.dir = 5
              else:
                 self.dir = 4
           elif self.left_vel_up < 0 and self.right_vel_up < 0:
              if self.left_vel_up == self.right_vel_up:
                 self.dir = 1
              elif self.left_vel_up > self.right_vel_up:
                 self.dir = 7
              else:
                 self.dir = 6
           elif self.left_vel_up > 0 and self.right_vel_up < 0:
              self.dir = 3
           elif self.left_vel_up < 0 and self.right_vel_up > 0:
              self.dir = 2



def main(args=None):
    rclpy.init(args=args)

    motor = Motor()

    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

