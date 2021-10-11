#!/usr/bin/env python
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

"""
def imuCallback(data):
    rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

def keyCallback(data):
    key = data.code
    rospy.loginfo("%i", key)

def main():
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('rosbot_turn', anonymous=True)
    rospy.Subscriber("/imu", Imu, imuCallback)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 1.0
    count = 0
    while not rospy.is_shutdown():
        vel_pub.publish(vel)
    	count+=1
        if(count==20):
            vel.linear.x = 0.0
        vel.angular.z = 0.0
        break
        rate.sleep()
if __name__ == "__main__":
    main()
"""


# range topic /range/fr/range
# range topic /range/fr/range
#/odom/twist/twist/angular/z
#/odom/twist/twist/linear/x
class shifty:
	def __init__(self):
		#initializing rosbot node as shifty
		rospy.init_node('shifty', anonymous=True)
		self.rateVal = 10
		self.dt = 0.1		
		self.rate = rospy.Rate(self.rateVal)

		#Setting up publishers
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)		
		
		#joystick values
		self.joyLeft_x = 0
		self.joyLeft_y = 0
		self.joyRight_x = 0
		self.joyRight_y = 0
		self.rightTrigger_down = 0
		self.rightTrigger_up = 0
		self.button_A = 0
		self.button_B = 0
		self.button_X = 0
		self.button_Y = 0


		#state values
		self.isCruiseModeOn = True

		#velocity values
		self.linear_velocity = 0
		self.angular_velocity = 0
		self.cruise_velocity = 0

		#initializing joystick
		self.init_joyStick()

		#inilializing irSensor
		self.init_irSensor()
		self.iterationsToRotateFor = 0
		
		#initializing the odom
		self.init_odom()
		self.current_linear_velocity = 0
		self.current_angular_velocity = 0
		self.prev_error = 0
		self.error = 0
		self.integral = 0
		
		# allow state variables to populate from subscribers
        	#rospy.sleep(1)
    	
	def init_odom(self):
		def odom_callback(data):
			self.current_linear_velocity = data.twist.twist.linear.x
			self.current_angular_velocity = data.twist.twist.angular.z
			
		rospy.Subscriber('/odom', Odometry, odom_callback)

	def init_irSensor(self):
		def ir_callback(data):
			rospy.loginfo("detected %f, number= %f", data.fl.range, self.iterationsToRotateFor)
			left_ir = data.fl.range
			right_ir = data.fr.range
			avg = (left_ir + right_ir)/2
			threshold = self.cruise_velocity * 15
			
			if(avg < threshold and self.iterationsToRotateFor == 0):
				self.linear_velocity = 0
				self.angular_velocity = 0
				self.setVelocity()
				self.iterationsToRotateFor = 30
				var = random.randint(0,1)*2 -1
				self.angular_velocity = var			
				
		rospy.Subscriber("/range", Range, ir_callback)
		#rospy.loginfo("subscribing")

	def init_joyStick(self):
		#defining the joystick callback
		def joystick_callback(data):
			#populating potentiometer data
			self.joyLeft_y = data.axes[1]
			self.joyRight_x = data.axes[2]
			self.rightTrigger_up = data.buttons[5] #check this once
			
			self.button_A = data.buttons[0]
			self.button_B = data.buttons[1]
			self.button_X = data.buttons[2]
			self.button_Y = data.buttons[3]

			#calling teleop function
			if(self.rightTrigger_up>0):
				self.isCruiseModeOn = False
				rospy.loginfo("right trig: %f", self.rightTrigger_up )
				#self.teleopSpeed()
			else:
				self.isCruiseModeOn = True
				self.cruiseMode()
				#self.setCruiseSpeed()				
			
			#logging the data for debugging
			#rospy.loginfo("y=%f, x=%f", self.joyLeft_y, self.joyRight_x)
			#rospy.loginfo(data.axes)
		
		#initializing subscriber and stating callback for joystick
		rospy.Subscriber("/joy", Joy, joystick_callback)

	#defing cruise mode speed
	def cruiseMode(self):
		if(self.button_A):
			self.cruise_velocity = 0.3
		elif(self.button_B):
			self.cruise_velocity = 0.5
		elif(self.button_X):
			self.cruise_velocity = 0.6
		elif(self.button_Y):
			self.cruise_velocity = 0

	#Setting the cruise speed based on joystick data
	def setCruiseSpeed(self):
		if(self.iterationsToRotateFor == 0):
			self.linear_velocity = self.cruise_velocity
			self.angular_velocity = 0
		else:
			self.linear_velocity = 0

	#Setting speed for teleoperation
	def teleopSpeed(self):
		#getting abs val for granular control
		absVal_y = abs(self.joyLeft_y)
		absVal_x = abs(self.joyRight_x)
		#Setting threshold and multipliers 
		threshold = 0.3
		multiplier1 = 0.75
		multiplier2 = 0.6
		multiplier3 = 1.2
		multiplier4 = 1.5
		if(absVal_y>threshold):
			self.linear_velocity = multiplier1 * self.joyLeft_y
		else:
			self.linear_velocity = multiplier2 * self.joyLeft_y

		if(absVal_x>threshold):
			self.angular_velocity = multiplier3 * self.joyRight_x
		else:
			self.angular_velocity = multiplier4 * self.joyRight_x
		
		#rospy.loginfo("Velocity X= %f, Y=%f", self.linear_velocity, self.angular_velocity)
		#self.setVelocity()

	def setVelocity(self):
		vel = Twist()
		vel.linear.x = self.linear_velocity
		vel.angular.z = self.angular_velocity
		rospy.loginfo("Velocity X= %f, Y=%f", vel.linear.x, vel.angular.z)
		self.vel_pub.publish(vel)	
	
	def pid(self, current, desired, kp=0.1, ki=0.001, kd = 0.03):
		#adsasdasd
		self.error = desired - current
		integral = self.integral +  self.error*self.dt
		derivative = (self.error - self.prev_error)/self.dt
		output = kp*self.error + ki*integral + kd*derivative
		
		self.prev_error = self.error
		self.integral = integral
		return output

	def runUsingJoy(self):
		rospy.loginfo("Velocity X= %f, Y=%f", self.vel.linear.y, self.vel.angular.z)
	
	def run(self):
		vel = Twist()
		while not rospy.is_shutdown():
			if(self.isCruiseModeOn):
				self.setCruiseSpeed()
			else:
				self.teleopSpeed()
				
			#vel.linear.x = self.linear_velocity 
			vel.linear.x = self.linear_velocity + self.pid(self.current_linear_velocity, self.angular_velocity)
			vel.angular.z = self.angular_velocity
			rospy.loginfo(self.pid(self.current_linear_velocity, self.angular_velocity))		
			#rospy.loginfo("Velocity X= %f, Y=%f", vel.linear.x, vel.angular.z)
			#rospy.loginfo("Joy data trig= %f, A=%f, B=%f, X=%f, Y=%f", self.rightTrigger_up, self.button_A, self.button_B, self.button_X, self.button_Y)
			self.vel_pub.publish(vel)
			if(self.iterationsToRotateFor >0):
				self.iterationsToRotateFor-=1			
			self.rate.sleep()
			

if __name__ == "__main__":
	sh = shifty()
	sh.run()

