#!/usr/bin/env python

import rospy
# import os
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class velocity_controller:
   """
   A node including call of initilization service, move command publisher, position subscriber and scan information subscriber
   """

   def __init__(self):

       # Initialize Publisher according to robot
       if rospy.get_param('~/ROBOT') == 'rto-1':
           self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
       elif rospy.get_param('~/ROBOT') == 'p3dx':
           self.move_pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)

       # Initialize Subscribers
       self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callback_move)

       # Initialize command type
       self.move_command_pub = Twist()

       # get initialized parameter
       self.vel_max = rospy.get_param('~/velocity_max')
       self.dis_stop = rospy.get_param('~/distance_stop')
       self.dis_slow = rospy.get_param('~/distance_slow')

   # callback of cmd_velocity listener and record initial velocity
   def callback_move(self, Twist):
       self.move_command_pub = Twist

   # transform laser input to position respect to sensor
   def get_distance(self):
       # receive laser information
       self.msg = rospy.wait_for_message("scan", LaserScan)

       # x,y position
       self.dis_y = []
       self.dis_x = []

       # transfrom laser information and determine option state
       for i in range(245):
           # laser range to x,y position
           disx = self.msg.ranges[i] * np.cos(i * 0.0163934417 - 2)
           disy = self.msg.ranges[i] * np.sin(i * 0.0163934417 - 2)
           self.dis_x.append(disx)
           self.dis_y.append(disy)

   # determine command
   def get_command(self):

       # check forward area
       if self.move_command_pub.linear.x > 0:
           x = []
           for i in range(81, 164):
               x.append(self.dis_x[i])
           mini_x = min(x)
           # stop
           if mini_x < self.dis_stop:
               self.move_command_pub.linear.x = 0
           # slow down
           elif mini_x < self.dis_slow:
               self.move_command_pub.linear.x = self.vel_max * ((mini_x - self.dis_stop)/(self.dis_slow - self.dis_stop))
           # back to maximal speed if there is no obstacle forward and x_velocity is not 0
           elif self.move_command_pub.linear.x != 0:
               self.move_command_pub.linear.x = self.vel_max
       # since the sensor cannot see backward, set it to 0
       elif self.move_command_pub.linear.x < 0:
           self.move_command_pub.linear.x = 0

       # check left side area
       if self.move_command_pub.linear.y > 0:
           y_left = []
           # check left side
           for i in range(164, 245):
               y_left.append(np.abs(self.dis_y[i]))
           mini_y_left = min(y_left)
           # stop
           if mini_y_left < self.dis_stop:
               self.move_command_pub.linear.y = 0
           # slow down
           elif mini_y_left < self.dis_slow:
               self.move_command_pub.linear.y = self.vel_max * ((mini_y_left - self.dis_stop)/(self.dis_slow - self.dis_stop))
           # back to maximal speed
           elif self.move_command_pub.linear.y != 0:
               self.move_command_pub.linear.y = self.vel_max
       # check right side area
       elif self.move_command_pub.linear.y < 0:
           y_right = []
           # check right side
           for i in range(0, 81):
               y_right.append(np.abs(self.dis_y[i]))
           mini_y_right = min(y_right)
           # stop
           if mini_y_right < self.dis_stop:
               self.move_command_pub.linear.y = 0
           # slow down
           elif mini_y_right < self.dis_slow:
              self.move_command_pub.linear.y = ( -self.vel_max) * ((mini_y_right - self.dis_stop)/(self.dis_slow - self.dis_stop))
           # back to maximal speed
           elif self.move_command_pub.linear.y != 0:
               self.move_command_pub.linear.y = -self.vel_max


   # Accordding to situation to publish adjusted move command
   def run(self, rate: float = 30):
       while not rospy.is_shutdown():

           # Apply distance check every step
           self.get_distance()
           self.get_command()

           # publish adjusted command
           self.move_pub.publish(self.move_command_pub)

           # if rate:
           #     rospy.sleep(1/rate)

if __name__ == "__main__":
   rospy.init_node('Velocity_Controller')

   velocity_controller = velocity_controller()
   velocity_controller.run(rate=30)
