"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""
import math
import time
import sys

import wpilib
import wpilib.command
import networktables

import const

class Intake(wpilib.command.Subsystem):
	def __init__(self, robot):
		super().__init__('intake')

		self.robot = robot

		self.motor_left = wpilib.VictorSP(const.PWM_MOTOR_INTAKE_LEFT)
		self.motor_right = wpilib.VictorSP(const.PWM_MOTOR_INTAKE_RIGHT)

		self.motor_group = wpilib.SpeedControllerGroup(self.motor_left, self.motor_right)

		self.rotation_motor = wpilib.VictorSP(const.PWM_MOTOR_ROTATION)

		self.intake_solenoid_1 = wpilib.Solenoid(const.CAN_PCM, const.PCM_INTAKE_SOLENOID_1)
		self.intake_solenoid_2 = wpilib.Solenoid(const.CAN_PCM, const.PCM_INTAKE_SOLENOID_2)

		self.intake_solenoid_1.set(True)
		self.intake_solenoid_2.set(False)

		self.has_cube = False

	def compress(self):
		self.intake_solenoid_1.set(True)
		self.intake_solenoid_2.set(False)

	def release(self):
		self.intake_solenoid_1.set(False)
		self.intake_solenoid_2.set(True)

	def set_speed(self, speed):
		if speed < 0:
			self.has_cube = False
		self.motor_group.set(speed)

	def stop(self):
		self.motor_group.set(0)

	def get_current(self):
		return self.robot.pdp.getCurrent(const.PDP_MOTOR_INTAKE)

	def set_rotation_speed(self, speed):
		self.rotation_motor.set(speed)

	def stop_rotation(self):
		self.rotation_motor.set(0)

	def log(self):
		pass