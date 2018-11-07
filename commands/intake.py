"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.command import Command
from wpilib import Timer

import networktables
import const
import time

import subsystems.intake


class Run_Intake(Command):
	def __init__(self, robot, speed = 0):
		super().__init__()
		
		self.robot = robot
		self.speed = speed
		
		self.requires(self.robot.intake)
		self.setInterruptible(True)

	def execute(self):
		self.robot.intake.set_speed(self.speed)

	def isFinished(self):
		return True


class Run_Intake_Controller(Command):
	def __init__(self, robot, speed, speed_reverse):

		super().__init__()

		self.robot = robot
		self.speed = speed
		self.speed_reverse = speed_reverse

		self.requires(self.robot.intake)
		self.setInterruptible(True)

	def execute(self):
		speed = self.speed() * 0.62
		speed_reverse = -self.speed_reverse()	
		
		if speed == 0 and speed_reverse == 0:
			self.robot.intake.stop()
		elif speed > 0:
			self.robot.intake.set_speed(speed)
		else:
			self.robot.intake.set_speed(speed_reverse)

	def isFinished(self):
		return False


class Run_Intake_Timeout(Command):
	def __init__(self, robot, speed, timeout):
		super().__init__()

		self.robot = robot
		self.speed = speed
		
		self.requires(self.robot.intake)
		self.setInterruptible(True)		
		self.setTimeout(timeout)

	def execute(self):
		self.robot.intake.set_speed(self.speed)

	def end(self):
		self.robot.intake.stop()

	def isFinished(self):
		timed_out = self.isTimedOut()
		return timed_out
	
	
class Run_Intake_Current(Command):
	def __init__(self, robot, speed, threshold = const.INTAKE_CURRENT_THRESHOLD, timeout = 5):
		super().__init__()

		self.robot = robot
		self.speed = speed
		self.threshold = threshold

		self.requires(self.robot.intake)
		self.setInterruptible(True)
		self.setTimeout(timeout)
		
	def initialize(self):
		self.startingTime = Timer.getFPGATimestamp()		
		self.robot.intake.has_cube = False

	def execute(self):
		self.robot.intake.set_speed(self.speed)

	def end(self):
		self.robot.intake.stop()

	def isFinished(self):
		timed_out = self.isTimedOut()
		current = self.robot.pdp.getCurrent(const.PDP_MOTOR_INTAKE)
		threshold_passed = current >= const.INTAKE_CURRENT_THRESHOLD
		threshold_time_passed = Timer.getFPGATimestamp() - self.startingTime >= 0.75

		return (timed_out or threshold_passed or self.robot.intake.has_cube) and threshold_time_passed


class Set_Cube_State(Command):
	def __init__(self, robot, state):
		super().__init__()

		self.robot = robot
		self.state = state

		self.requires(self.robot.intake)
		self.setInterruptible(False)

	def execute(self):
		self.robot.intake.has_cube = self.state

	def isFinished(self):
		return True


class Stop_Intake(Run_Intake):
	def execute(self):
		self.robot.intake.stop()

		
class Set_Rotation_Speed(Run_Intake):
	def execute(self):
		self.robot.intake.set_rotation_speed(self.speed)

		
class Stop_Rotation(Run_Intake):
	def execute(self):
		self.robot.intake.stop_rotation()

		
class Set_Intake_Compression(Command):
	def __init__(self, robot, state):

		super().__init__()

		self.robot = robot
		self.state = state

		self.requires(self.robot.intake)
		self.setInterruptible(True)

	def execute(self):
		if self.state:
			self.robot.intake.compress()
		else:
			self.robot.intake.release()
		
	def isFinished(self):
		return True