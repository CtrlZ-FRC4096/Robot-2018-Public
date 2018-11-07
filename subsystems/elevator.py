"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

import math
import sys
import time

import wpilib
import ctre
import networktables

import const


class Elevator(wpilib.command.Subsystem):
	def __init__(self, robot):
		"""
		Initializes the elevator
		"""
		super().__init__('elevator')
		self.robot = robot

		# Set up electrical components
		self.talon = ctre.WPI_TalonSRX(const.CAN_ELEVATOR_TALON)
		self.victor_spx = ctre.WPI_VictorSPX(const.CAN_ELEVATOR_VICTOR_SPX)

		self.elevator_solenoid_1 = wpilib.Solenoid(const.CAN_PCM, const.PCM_ELEVATOR_BRAKE_SOLENOID_1)
		self.elevator_solenoid_2 = wpilib.Solenoid(const.CAN_PCM, const.PCM_ELEVATOR_BRAKE_SOLENOID_2)

		self.elevator_solenoid_1.set(False)
		self.elevator_solenoid_2.set(True)

		self.kF = 0.02
		self.kP = 0.02
		self.kI = 0.0
		self.kD = 0.0
		self.kTimeoutMs = 10

		self.talon.selectProfileSlot(0, 0)
		self.talon.config_kF(0, self.kF, self.kTimeoutMs)
		self.talon.config_kP(0, self.kP, self.kTimeoutMs)
		self.talon.config_kI(0, self.kI, self.kTimeoutMs)
		self.talon.config_kD(0, self.kD, self.kTimeoutMs)

		self.talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
		self.talon.setSelectedSensorPosition(0, 0, self.kTimeoutMs)

		self.talon.configAllowableClosedloopError(0, 5000, self.kTimeoutMs)

		self.talon.setStatusFramePeriod(self.talon.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, self.kTimeoutMs)
		self.talon.setStatusFramePeriod(self.talon.StatusFrameEnhanced.Status_10_MotionMagic, 10, self.kTimeoutMs)

		self.talon.configNominalOutputForward(0, self.kTimeoutMs)
		self.talon.configNominalOutputReverse(0, self.kTimeoutMs)
		self.talon.configPeakOutputForward(1, self.kTimeoutMs)
		self.talon.configPeakOutputReverse(-1, self.kTimeoutMs)

		self.talon.setSensorPhase(True)
		self.talon.setInverted(False)

		self.talon.configMotionCruiseVelocity(200000, self.kTimeoutMs)	# ws 300000
		self.talon.configMotionAcceleration(125000, self.kTimeoutMs)

		self.position = 0

	def go_magic_distance(self, distance):
		# Uses motion magic for speed control -- prevents frying motors
		self.talon.set(ctre.WPI_TalonSRX.ControlMode.MotionMagic, distance)
		self.victor_spx.follow(self.talon)

	def run_elevator(self, value):
		self.talon.set(value * const.ELEVATOR_SPEED_MULTIPLIER)
		self.victor_spx.set(value * const.ELEVATOR_SPEED_MULTIPLIER)

	def stop_elevator(self):
		self.run_elevator(0)

	def set_brake(self):
		self.elevator_solenoid_1.set(True)
		self.elevator_solenoid_2.set(False)

	def release_brake(self):
		self.elevator_solenoid_1.set(False)
		self.elevator_solenoid_2.set(True)

	def set_pid_output(self, output):
		# Make sure we're outputing at least enough to move the elevator at all
		min_output = const.ELEVATOR_MIN

		if output < 0:
			output = max(-0.5, output)
		else:
			output = min(0.5, output)

		if self.pid.onTarget():
			output = 0

		self.pid_output = output

		self.run_elevator(self.pid_output)

	def update_pid(self, p = None, i = None, d = None):
		# Updates the PID values
		if p:
			self.kP = p
		if i:
			self.kI = i
		if d:
			self.kD = d

		self.pid.setPID(self.kP, self.kI, self.kD)

	def get_pid_input(self):
		return self.talon.getPulseWidthPosition()

	def log(self):
		wpilib.SmartDashboard.putData(self.talon)