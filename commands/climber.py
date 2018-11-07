"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.command import Command

import networktables
import const

import subsystems.climber

class Run(Command):
	def __init__(self, robot, speed):
		super().__init__()
		
		self.robot = robot
		self.speed = speed
		
		self.requires(self.robot.climber)
		self.setInterruptible(True)

	def execute(self):
		self.robot.climber.set_speed(self.speed)

	def isFinished(self):
		return True

class Stop(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot
		
		self.requires(self.robot.climber)
		self.setInterruptible(True)
		
	def execute(self):
		self.robot.climber.stop()

	def isFinished(self):
		return True
		
class Climber_Deploy(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot

		self.requires(self.robot.climber)
		self.setInterruptible(True)

	def execute(self):
		self.robot.climber.deploy()

	def isFinished(self):
		return True