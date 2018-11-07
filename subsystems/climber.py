"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""
import wpilib
import wpilib.command
import networktables

import const

### CLASSES ###

class Climber(wpilib.command.Subsystem):

	def __init__(self, robot):

		super().__init__('climber')

		self.robot = robot
		
		self.motor = wpilib.VictorSP(const.PWM_CLIMBER_MOTOR)
		
		self.motor.setInverted(True)
		
		self.servo = wpilib.Servo(const.PWM_CLIMBER_SERVO)

		self.is_released = False

	def set_speed(self, speed):
		self.motor.set(speed)
		
	def stop(self):
		self.set_speed(0)

	def set_servo_angle(self, angle):
		self.servo.setAngle(angle)

	def deploy(self):
		self.is_released = True
		print("Deploying")
		print(self.servo.getAngle())
		self.set_servo_angle(const.SERVO_RELEASE_ANGLE)
		print(self.servo.getAngle())

	def check_deploy_done(self):
		if self.servo.getAngle() - const.SERVO_RELEASE_ANGLE <= 15:
			return True
		else:
			return False
			
	def log(self):
		wpilib.SmartDashboard.putData(self)