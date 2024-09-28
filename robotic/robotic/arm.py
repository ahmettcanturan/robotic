import rclpy
import logging
from rclpy.node import Node
from math import atan2, sqrt, cos, sin
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        #self.Rmax = 743.0
        #self.Rmin = 284.0
        self.Rval = 0.0
        self.Px = 500.0
        self.Py = 0.0
        self.t1 = 0.0
        self.timer_period = 1.0 / 25.0
        self.gain = 1.0
        self.axis0 = 0.0
        self.axis1 = 0.0
        self.axis2 = 0.0
        self.axis4 = 0.0
        self.axis5 = 0.0
        self.grip_o = 0.0
        self.grip_c = 0.0
        self.grip_h = 0.0
        self.buton2 = 0.0
        self.buton3 = 0.0

        self.drive_axis0 = 0.0
        self.drive_axis1 = 0.0
        self.drive_axis3 = 0.0

        self.button7_prev = False
        self.button8 =  False
        self.button9 =  False
        self.button10 = False
        self.button11 = False

        self.button8_prev=  False
        self.button9_prev=  False
        self.button10_prev = False
        self.button11_prev = False
        self.state = True
        # self.state2 = True
        # self.state3 = True
        # self.state4 = True
        # self.state5 = True

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.arm_publisher = self.create_publisher(Twist, 'cmd_vel_arm', 10)
        self.drive_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vector_publisher = self.create_publisher(Vector3, 'servo_angles', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        # Assuming axes[0] is Px and axes[1] is Py
        self.gain = (msg.axes[3] + 1) * 20
        self.axis0 = msg.axes[1] * self.timer_period * self.gain * 3.0
        self.axis1 = msg.axes[0] * self.timer_period * self.gain * 3.0
        self.axis2 = msg.axes[2] * self.timer_period * self.gain / 75.0
        self.axis4 = msg.axes[4] * 100.0
        self.axis5 = msg.axes[5] * 100.0
        self.grip_o = msg.buttons[2]  * 1.0
        self.grip_c = msg.buttons[3] * 1.0
        self.grip_h = msg.buttons[0] * 1.0

        self.drive_axis0 = msg.axes[0]
        self.drive_axis1 = msg.axes[1]
        self.drive_axis3 = msg.axes[3]

        self.button8 = msg.buttons[8]
        self.button9 = msg.buttons[9]
        self.button10 = msg.buttons[10]
        self.button11 = msg.buttons[11]
        
        if msg.buttons[7] == True and msg.buttons[7] != self.button7_prev:
            self.state = not self.state
        self.button7_prev = msg.buttons[7]
        
        # if msg.buttons[10] == True:
        #     self.state2 = not self.state2
        
        # if msg.buttons[11] == True:
        #     self.state3 = not self.state3

        # if msg.buttons[8] == True:
        #     self.state4 = not self.state4

        # if msg.buttons[9] == True:
        #     self.state5 = not self.state5

    def timer_callback(self):
        vector_msg = Vector3()
        if self.state == False :
            self.Px += self.axis0
            self.Py += self.axis1
            self.t1 += self.axis2

            linear_axis = 0.0
            angular_axis = 0.0
            if self.button9 ==  True :
                linear_axis = 1.0
            elif self.button10 == True :
                linear_axis = -1.0

            if self.button11 ==  True :
                angular_axis = -1.0
            elif self.button8 == True :
                angular_axis = 1.0
            self.calculate_speed(axis0=angular_axis, axis1=linear_axis, axis3=-10)
        else :
            self.calculate_speed(axis0=self.drive_axis0, axis1=self.drive_axis1, axis3=self.drive_axis3)
            
        self.calculate_angles(Px=self.Px, Py=self.Py, t1=self.t1, t4=self.axis5, t5=self.axis4, t6=self.grip_o, t7=self.grip_c, t8=self.grip_h)

        #Alper bu değerleri hatırlamadım sonra ayarlamamız lazım

        # if self.state2 == False:
        #     vector_msg.x = 2550.0
        # else:
        #     vector_msg.x = 1000.0
        
        # if self.state3 == False:
        #     vector_msg.y = 2550.0
        # else:
        #     vector_msg.y = 1000.0

        # if self.state4 == False:
        #     vector_msg.z = 1400.0
        # elif self.state5 == False:
        #     vector_msg.z = 1600.0
        # else:
        #     vector_msg.z = 1500.0
    
        

        self.vector_publisher.publish(vector_msg)


    def calculate_speed(self, axis0, axis1, axis3):
        multiplier = (axis3 + 1.0) * 8000
        self.publish_cmd_vel(linear_x=(axis1 * multiplier), angular_z=(axis0 * multiplier))

    def publish_cmd_vel(self, linear_x, angular_z):
        twist_drive = Twist()

        twist_drive.linear.x = linear_x
        twist_drive.angular.z = angular_z
        #twist.linear.z = tz

        if self.state == True :
            lrpms = "{:.0f}".format(linear_x)
            arpms = "{:.0f}".format(angular_z)
            gs = "{:.2f}".format((self.drive_axis3 + 1.0) / 2.0)
            print(f'Joystick =>     Linear Rpm: {lrpms},  Angular Rpm: {arpms}, Gain: {gs}   |   Mode: Drive')


        self.drive_publisher.publish(twist_drive)


    def calculate_angles(self, Px, Py, t1, t4, t5, t6, t7, t8):
        a1 = 500
        a2 = 300
        try:
            cost3 = (Px**2 + Py**2 - a1**2 - a2**2) / (2 * a1 * a2)
            # Clamping cost3 to be within [-1, 1] to avoid math domain errors
            cost3 = max(min(cost3, 1), -1)
            sint3 = -1 * sqrt(1 - cost3**2)
            t3 = atan2(sint3, cost3)

            determinant = ((a1 + a2 * cos(t3))**2) + ((a2 * sin(t3))**2)
            cost2 = (Px * (a1 + a2 * cos(t3)) - (-Py * a2 * sin(t3)))
            sint2 = (Py * (a1 + a2 * cos(t3)) - (Px * a2 * sin(t3)))
            t2 = atan2(sint2 / determinant, cost2 / determinant)
            t4 = t4 
            t5 = t5 
            t6 = t6
            t7 = t7
            t8 = t8 * 0.5
            if self.state == False :
                mode = 'Arm'
                pxs = "{:.2f}".format(Px)
                pys = "{:.2f}".format(Py)
                t1s = "{:.2f}".format(t1)
                gs = "{:.2f}".format((self.drive_axis3 + 1.0) / 2.0)
                print(f'Joystick =>     Px: {pxs}, Py: {pys}, t1: {t1s},  Gain: {gs}   |   Mode: {mode}')
                
            
            self.publish_cmd_vel_arm(t1, t2, t3, t4, t5, t6, t7, t8)

        except ValueError as e:
            self.get_logger().error(f'Error in computing angles: {e}')

    def publish_cmd_vel_arm(self, t1, t2, t3, t4, t5, t6, t7, t8):
        twist_arm = Twist()
        if t6 > 0:
            twist_arm.linear.z = 100* t6 
        elif t7 > 0:
            twist_arm.linear.z = -100* t7 
        elif t8 > 0:
            twist_arm.linear.z = 100*t8
            
        twist_arm.angular.x = -t2-t3
        twist_arm.angular.y = t2
        twist_arm.angular.z = t1
        twist_arm.linear.x = t4 + t5
        twist_arm.linear.y = t4 - t5
        #twist.linear.z = tz

        self.arm_publisher.publish(twist_arm)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()