# -*- coding: utf-8 -*-
"""
Created on Mon Oct 11 16:45:39 2021

@author: Ashish
"""
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class shifty:
  def __init__(self):
    rospy.init_node('shifty', anonymous=True)
    self.rateVal = 10
    self.dt = 0.1
    self.rate = rospy.Rate(self.rateVal)
    
    #Setting up publishers
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)	
		
    #velocity values
    self.linear_velocity = 0
    self.angular_velocity = 0
    
    #Robot State
    self.current_linear_velocity = 0
    self.current_angular_velocity = 0
    self.current_position_x = 0
    self.current_position_y = 0
    self.current_position_theta = 0
    
    self.init_odom()
        
        
    def setVelocity(self):
      vel = Twist()
      vel.linear.x = self.linear_velocity
      vel.angular.z = self.angular_velocity
      rospy.loginfo("Velocity X= %f, Y=%f", vel.linear.x, vel.angular.z)
      self.vel_pub.publish(vel)
    
    def init_odom(self):
      def odom_callback(data):
        self.current_linear_velocity = data.twist.twist.linear.x
        self.current_angular_velocity = data.twist.twist.angular.z
        self.current_position_x = data.pose.pose.orientation.x
        self.current_position_y = data.pose.pose.orientation.y
        self.current_position_theta = euler_from_quaternion([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w])[2]
  
      rospy.Subscriber('/odom', Odometry, odom_callback)
    
    def go_to_goal(self,final_x,final_y):
      velocity_gain = 0.2
      steering_gain = 0.1
      
      theta_goal = math.atan((final_y - self.current_position_y)/(final_x - self.current_position_x))
      theta_err = theta_goal - self.current_position_theta
      
      distance = math.sqrt((final_x - self.current_position_x)**2 + (final_y - self.current_position_y)**2)
      velocity = velocity_gain*distance
      
      if velocity>1:
        velocity = 1
      
      self.linear_velocity = velocity
      self.angular_velocity = steering_gain*theta_err
      
    
    def run(self):
      vel = Twist()
		while not rospy.is_shutdown():
          self.go_to_goal(1.5,1.5)
          self.setVelocity()
          
if __name__ == "__main__":
	sh = shifty()
	sh.run()