#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
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

class shifty:
	def __init__(self):
		#initializing rosbot node as shifty
		rospy.init_node('shifty', anonymous=True)
		rate = rospy.Rate(10)

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
		
		# allow state variables to populate from subscribers
        	#rospy.sleep(1)
    
	def init_joyStick(self):
		#defining the joystick callback
		def joystick_callback(data):
			#populating potentiometer data
			self.joyLeft_y = data.axes[1]
			self.joyRight_x = data.axes[3]
			self.rightTrigger_down = data.axes[4] #check this once
			self.button_A = data.button[0]
			self.button_B = data.button[1]
			self.button_X = data.button[2]
			self.button_Y = data.button[3]

			#calling teleop function
			if(self.rightTrigger_down):
				self.isCruiseModeOn = False
				self.teleopSpeed()
			else:
				self.isCruiseModeOn = True
				self.cruiseMode()
				self.setCruiseSpeed()
			
			#logging the data for debugging
			rospy.loginfo("y=%f, x=%f", self.joyLeft_y, self.joyRight_x)
			rospy.loginfo(data.axes)
		
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
		self.linear_velocity = self.cruise_velocity
		self.setVelocity()	

	#Setting speed for teleoperation
	def teleopSpeed(self):
		#getting abs val for granular control
		absVal_y = abs(self.joyLeft_y)
		absVal_x = abs(self.joyRight_x)
		#Setting threshold and multipliers 
		threshold = 0.3
		multiplier1 = 0.75
		multiplier2 = 0.6
		if(absVal_y>threshold):
			self.linear_velocity = multiplier1 * self.joyLeft_y
		else:
			self.linear_velocity = multiplier2 * self.joyLeft_y

		if(absVal_x>threshold):
			self.angular_velocity = multiplier1 * self.joyRight_x
		else:
			self.angular_velocity = multiplier2 * self.joyRight_x
		
		#rospy.loginfo("Velocity X= %f, Y=%f", self.linear_velocity, self.angular_velocity)
		self.setVelocity()

	def setVelocity(self):
		vel = Twist()
		vel.linear.x = self.linear_velocity
		vel.angular.z = self.angular_velocity
		rospy.loginfo("Velocity X= %f, Y=%f", vel.linear.x, vel.angular.z)
		self.vel_pub.publish(vel)	

	def runUsingJoy(self):
		rospy.loginfo("Velocity X= %f, Y=%f", self.vel.linear.y, self.vel.angular.z)
	
	def run(self):
		while not rospy.is_shutdown():
			#self.setVelocity()
			rospy.loginfo("HI")
			

if __name__ == "__main__":
	sh = shifty()
	sh.run()

