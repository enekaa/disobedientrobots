import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from .submodules.Motor import *
import RPi.GPIO as GPIO

IRO1 = 14
IRO2 = 15
IRO3 = 23

class LineFollower(Node):
	def __init__(self):
		super().__init__("line_tracker_node")
		self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
		self.timer_ = self.create_timer(0.1, self.callback)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(IRO1, GPIO.IN)
		GPIO.setup(IRO2, GPIO.IN)
		GPIO.setup(IRO3, GPIO.IN)
	def callback(self):
		msg = String()
		LMR = 0x00
		if GPIO.input(IRO1) == True:
			LMR = (LMR | 4)
		if GPIO.input(IRO2) == True:
			LMR = (LMR | 2)
		if GPIO.input(IRO3) == True:
			LMR = (LMR | 1)
		
		if LMR == 2:
			PWM.setMotorModel(800,800,800,800)
		elif LMR == 4:
			PWM.setMotorModel(-1500,-1500,2500,2500)
		elif LMR == 6:
			PWM.setMotorModel(-2000,-2000,4000,4000)
		elif LMR == 1:
			PWM.setMotorModel(2500,2500,-1500,-1500)
		elif LMR == 3:
			PWM.setMotorModel(4000,4000,-2000,-2000)
		elif LMR == 7:
			pass
		
		self.publisher_.publish(msg)

#if __name__ == '__main__':
	#print("Program Line Follower is running ... ")
	#try:
	#	infrared.run()
	#except KeyboardInterrupt:
	#	PWM.setMotorModel(0,0,0,0)

def main(args=None):
	rclpy.init(args=args)
	node = LineFollower()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	

if __name__ == '__main__':
	print("Program Line Follower is running ... ")
	try:
		main()
	except KeyboardInterrupt:
		PWM.setMotorModel(0,0,0,0)
