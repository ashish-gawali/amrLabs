# -*- coding: utf-8 -*-
"""
Created on Wed Sep  8 17:53:40 2021

@author: Ashish
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

def imuCallback(data):
  rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

def main():
  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  rospy.init_node(‘rosbot_turn', anonymous=True)
  rospy.Subscriber("/imu", Imu, imuCallback)
  rate = rospy.Rate(10)
  vel = Twist()
  vel.linear.x = 0.5
  vel.angular.z = 1.0
  count = 0
  while not rospy.is_shutdown():
    vel_pub.publish(vel)
    rate.sleep()
    count+=1
    if(count==20):
      vel.linear.x = 0.0
      vel.angular.z = 0.0

if __name__ == "__main__":
  main()
