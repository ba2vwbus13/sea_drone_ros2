#!/usr/bin/env python
import pigpio
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
import datetime

class Thruster(Node):
    def __init__(self):
        super().__init__('thruster')
        self.pwm_frequency = 50#Hz
        self.output_pin_L = 13 #for raspi
        self.output_pin_R = 12 #for raspi
        self.pwm_L = pigpio.pi()
        self.pwm_R = pigpio.pi()
        self.pwm_L.hardware_PWM(self.output_pin_L, self.pwm_frequency, self.ratio_to_duty(0.5))#0=forward 0.5=stop 1=backward
        self.pwm_R.hardware_PWM(self.output_pin_R, self.pwm_frequency, self.ratio_to_duty(0.5))#0=forward 0.5=stop 1=backward

        self.previous_control_time = datetime.datetime.now()
        self.subscription = self.create_subscription(String, '/vertial_joy', self.joy_callback, 1)
        self.create_timer(0.1, self.change_speed) #0.1:if wait is too short, motor does not move
        self.linear_L = 0
        self.linear_R = 0 

    def joy_callback(self, joy_msg):
        self.linear_L = joy_msg.linear_L
        self.linear_R = joy_msg.linear_R
        self.get_logger().info(f"(joy) left = {self.linear_L} right = {self.linear_R}")

    def ratio_to_duty(self, ratio):
        ratio = max(min(ratio, 0.7), 0.3)
        #https://discuss.bluerobotics.com/t/controlling-esc-with-jetson-xavier-python/10013/2
        #duty = (1100+(ratio*800))/1000000 * self.pwm_frequency * 100
        duty = (1900-(ratio*800))/1000000 * self.pwm_frequency * 100
        duty = int(duty*10000)
        self.get_logger().info(f"ratio = {ratio} duty = {duty}")
        return duty

    def change_speed(self):
        ratio = (self.linear_L+1.0)/2.0
        self.get_logger().info(f"pwm Left {ratio}")
        self.pwm_L.hardware_PWM(self.output_pin_L, self.pwm_frequency, self.ratio_to_duty(ratio))#0=forward 0.5=stop 1=backward

        ratio = (self.linear_R+1.0)/2.0
        self.get_logger().info(f"pwm Right {ratio}")
        self.pwm_R.hardware_PWM(self.output_pin_R, self.pwm_frequency, self.ratio_to_duty(ratio))#0=forward 0.5=stop 1=backward        

        
    def shutdown_callback(self):
        self.get_logger().info('Stop')
        self.pwm_L.hardware_PWM(self.output_pin_L, self.pwm_frequency, self.ratio_to_duty(0.5))#0=forward 0.5=stop 1=backward
        self.pwm_R.hardware_PWM(self.output_pin_R, self.pwm_frequency, self.ratio_to_duty(0.5))#0=forward 0.5=stop 1=backward     
        self.pwm_L.stop()
        self.pwm_R.stop()
        rclpy.sleep(1)
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    thruster = Thruster()
    rclpy.spin(thruster)
    rclpy.shutdown(thruster.shutdown_callback)

if __name__ == '__main__':
    main()