#!/usr/bin/env python3

#あらかじめ実行すること
#(1) ~/RaspberryPiMouse/utils/build_install.bash
#(2) ros2 launch raspimouse raspimouse.launch.py

from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import datetime

class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')

        self.previous_control_time = datetime.datetime.now()
        self.linear_R = 0.0
        self.linear_L = 0.0
        self.subscription = self.create_subscription(String, 'controller', self.set_joy, 1)
        self.pub = self.create_publisher(Twist, '/verchal_joy', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    def set_joy(self, data):
        rev = data.data.split(',')
        speed = float(rev[0])
        angle = float(rev[1])
        self.linear_R = (angle*WHEEL_DIST)/2 + speed
        self.linear_L = speed*2-self.linear_R
        self.previous_control_time = datetime.datetime.now()

    def run(self, reset=False):
        if datetime.datetime.now() > self.previous_control_time + datetime.timedelta(seconds=0.3):
            self.linear_R = 0.0
            self.linear_L = 0.0
        msg.data = "{},{}".format(self.linear_R, self.linear_L) 
        self.get_logger().info(msg.data)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)

if __name__ == '__main__':
    main()