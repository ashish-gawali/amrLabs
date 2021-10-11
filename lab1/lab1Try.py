# -*- coding: utf-8 -*-
"""
Created on Wed Sep  8 17:55:34 2021

@author: Ashish
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range


class shifty:
  def __init__(self):
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('shifty', anonymous=True)
    self.joy_sub = rospy.Subscriber("/joy", Joy, joyCallback)
    self.joyLeft_x
    self.joyLeft_y
    self.joyRight_x
    self.joyRight_y
    
  def joyCallback(self, data):
    self.joyLeft_x = data.axes[0];
    self.joyLeft_y = data.axes[1];
    self.joyRight_x = data.axes[2];
    self.joyRight_y = data.axes[4];
    
  def runUsingJoy(self):
    absVal_y = self.joyLeft_y
    absVal_x = self.joyRight_x
    
    #Setting threshold and multipliers 
    threshold = 0.3
    multiplier1 = 1.5
    multiplier2 = 1.2
    vel = Twist()

    if(absVal_y>threshold):
      vel.linear.y = multiplier1 * self.joyLeft_y
    else:
      vel.linear.y = multiplier2 * self.joyLeft_y
    
    if(absVal_x>threshold):
      vel.linear.x = multiplier1 * self.joyRight_x
    else:
      vel.linear.x = multiplier2 * self.joyRight_x
    
    self.vel_pub.publish(vel)

if __name__ == "__main__":
  sh = shifty()
  sh.runUsingJoy()
  
