#! python3

"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

import logging
import os
import sys
import time
import math

import wpilib
import wpilib.sendablechooser
import wpilib.smartdashboard
import wpilib.command
import wpilib.timer

import networktables
import ctre

import robotpy_ext.control.xbox_controller

import controls.xbox_controller
import oi
import const
import commands.drivetrain
import commands.autonomous

import subsystems.drivetrain
import subsystems.elevator
import subsystems.intake
import subsystems.climber

log = logging.getLogger('robot')


class Robot(wpilib.IterativeRobot):
	"""
	Main robot class.

	This is the central object, holding instances of all the robot subsystem
	and sensor classes.

	It also contains the init & periodic methods for autonomous and
	teloperated modes, called during mode changes and repeatedly when those
	modes are active.

	The one instance of this class is also passed as an argument to the
	various other classes, so they have full access to all its properties.
	"""
	def robotInit(self):

		### Subsystems ###

		# Practice bot digital pin jumper checker
		# True when jumper is not in robot
		self.is_pbot = not wpilib.DigitalInput(const.DIO_PBOT_CHECK).get()

		# Drive for Xbox Controller
		self.drive = subsystems.drivetrain.Drivetrain(self)

		# Robot Components
		self.elevator = subsystems.elevator.Elevator(self)
		self.intake = subsystems.intake.Intake(self)
		self.climber = subsystems.climber.Climber(self)

		# Gyro
		self.gyro = wpilib.ADXRS450_Gyro()

		# Autonomous
		self.auto_run = False
		self.auto_choose = wpilib.sendablechooser.SendableChooser()
		wpilib.SmartDashboard.putData("Autonomous", self.auto_choose)
		self.game_data = ''

		# Operator Input
		self.oi = oi.OI(self)

		# Power Distribution Panel
		self.pdp = wpilib.PowerDistributionPanel()

		# Camera Server
		wpilib.CameraServer.launch()

		# Driver Station
		self.ds = wpilib.DriverStation.getInstance()

		# Encoders
		self.use_enc_correction = False

		self.drive_encoder_left = wpilib.Encoder(const.DIO_DRIVE_ENC_LEFT_1, const.DIO_DRIVE_ENC_LEFT_2, reverseDirection = const.DRIVE_ENCODER_LEFT_REVERSED)
		self.drive_encoder_right = wpilib.Encoder(const.DIO_DRIVE_ENC_RIGHT_1, const.DIO_DRIVE_ENC_RIGHT_2, reverseDirection = const.DRIVE_ENCODER_RIGHT_REVERSED)

		self.drive_encoder_left.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_FOOT)
		self.drive_encoder_right.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_FOOT)

		# Pressure sensor (200 psi)
		self.pressure_sensor = wpilib.AnalogInput(const.AIN_PRESSURE_SENSOR)

		# Timer for pressure sensor's running average
		self._pressure_samples = []
		self._last_pressure_value = 0.0
		self.pressure_timer = wpilib.Timer()
		self.pressure_timer.start()
		self.pressure_timer_delay = 1.0		# once per second

		# Time robot object was created
		self.start_time = time.time()

		## Scheduler ##
		self.scheduler = wpilib.command.Scheduler.getInstance()

		## Logging ##
		# Timers for NetworkTables update so we don't use too much bandwidth
		self.log_timer = wpilib.Timer()
		self.log_timer.start()
		self.log_timer_delay = 0.1		# 10 times/second

		self.log()

		# NetworkTables
		self.nt_smartdash = networktables.NetworkTables.getTable('SmartDashboard')

	### Disabled ###

	def disabledInit(self):
		self.climber.servo.setDisabled()
		self.log()
		wpilib.command.Scheduler.getInstance().removeAll()

	def disabledPeriodic(self):
		pass

	### Autonomous ###

	def autonomousInit(self):
		# Resets robot before running autonomous
		self.gyro.reset()
		self.elevator.talon.setSelectedSensorPosition(0, 0, 10)
		self.climber.servo.setAngle(0)

		const.DRIVE_CORRECTION_ENABLED = False

		# Retrives game specific data from the Field Management System
		self.game_data = self.ds.getGameSpecificMessage()
		self.auto_run = False
		self.use_enc_correction = False

		# Get the driver-selected auto mode from SmartDashboard and start it
		self.drive.set_gear_state(const.ID_HIGH_GEAR)
		self.selectedMode = self.auto_choose.getSelected()
		self.splitStrMode = str(self.selectedMode).split('_')

	def autonomousPeriodic(self):
		# Get game data from drivers station if it is not present
		if not self.game_data:
			self.game_data = self.ds.getGameSpecificMessage()

		# Checks to see if auto_run is false -- runs auto if false
		if not self.auto_run:
			print('Running Auto!')

			if self.game_data:
				self.auto_run = True  							# Only runs this block once -- bulk of autonomous code

				if self.splitStrMode[0] == "Cross":				# Cross baseline
					self.scheduler.add(commands.autonomous.Cross_Baseline(self))

				elif self.splitStrMode[2] == "Scale":			# Moves to scale
					print("Scale!")

					if self.game_data[1] == 'R':				# If scale is on the right side

						if self.splitStrMode[0] == "Left":		# And if we are on the left side
							if self.game_data[0] == 'L':		# If scale is on the right, but switch is on the left
								self.scheduler.add(commands.autonomous.Left_To_Switch_Left(self))
								print("Left to Switch Left")
							else:								# If scale and switch are both on the right
								self.scheduler.add(commands.autonomous.Cross_Baseline(self))
								print("Cross Baseline")

						#if self.splitStrMode[0] == "Left":
							#self.scheduler.add(commands.autonomous.Left_To_Scale_Right(self))
							#print("Left to Right Scale")

						elif self.splitStrMode[0] == "Right":
							self.scheduler.add(commands.autonomous.Right_To_Scale_Right(self))
							print("Right to Right Scale")

					elif self.game_data[1] == 'L':				# If scale is on the left side
						if self.splitStrMode[0] == "Left":		# and IF we are on the left side
							self.scheduler.add(commands.autonomous.Left_To_Scale_Left(self))
							print("Left to Left Scale")
						elif self.splitStrMode[0] == "Right":	# IF we are on the right side
							if self.game_data[0] == 'R':
								self.scheduler.add(commands.autonomous.Right_To_Switch_Right(self))
								print("Right to Switch Right")
							else:
								self.scheduler.add(commands.autonomous.Cross_Baseline(self))
								print("Cross Baseline")

				elif self.splitStrMode[2] == "Switch":			# IF we want switch
					print("Switch!")

					if self.game_data[0] == 'R':
						if self.splitStrMode[0] == "Left":
							#print("Left to Switch Right")
							#self.scheduler.add(commands.autonomous.Left_To_Switch_Right(self))
							print("Cross Baseline")
							self.scheduler.add(commands.autonomous.Cross_Baseline(self))
						elif self.splitStrMode[0] == "Right":
							print("Right to Right Switch")
							self.scheduler.add(commands.autonomous.Right_To_Switch_Right(self))
						elif self.splitStrMode[0] == "Center":
							print("Center to Right Switch")
							self.scheduler.add(commands.autonomous.Center_To_Switch_Right(self))

					elif self.game_data[0] == 'L':
						if self.splitStrMode[0] == "Left":
							print("Left to Left Switch")
							self.scheduler.add(commands.autonomous.Left_To_Switch_Left(self))
						elif self.splitStrMode[0] == "Right":
							print("Right to Left Switch")
							self.scheduler.add(commands.autonomous.Right_To_Switch_Left(self))
						elif self.splitStrMode[0] == "Center":
							print("Center to Left Switch")
							self.scheduler.add(commands.autonomous.Center_To_Switch_Left(self))

				else:
					print("Weird mode selected...")
					self.selectedMode.run()

		wpilib.command.Scheduler.getInstance().run()
		self.log()

	### Teleoperated ###

	def teleopInit(self):
		self.use_enc_correction = True
		# const.DRIVE_CORRECTION_ENABLED = True
		self.intake.has_cube = False

		# Removes any leftover auto commands from the scheduler
		wpilib.command.Scheduler.getInstance().removeAll()

		# Resetting robot parts
		self.gyro.reset()
		if not self.ds.isFMSAttached():
			self.elevator.talon.setSelectedSensorPosition(0, 0, 10)

		self.climber.servo.setAngle(0)

		self.drive_encoder_left.reset()
		self.drive_encoder_right.reset()

	def teleopPeriodic(self):
		wpilib.command.Scheduler.getInstance().run()

		self.log()

	### Misc ###

	def get_pressure(self):
		"""
		Calculate a running average of pressure values.  The sensor seems to jitter its values
		a lot, so this should smooth it out and make the SD display more readable.
		"""

		voltage_pressure = self.pressure_sensor.getVoltage()
		new_value = (250 * voltage_pressure / 5) - 25

		self._pressure_samples.append(new_value)

		if not self.pressure_timer.hasPeriodPassed(self.pressure_timer_delay):
			return self._last_pressure_value

		# Calculate new running average
		new_avg = sum(self._pressure_samples) / len(self._pressure_samples)

		self._pressure_samples = [ ]
		self._last_pressure_value = new_avg

		return new_avg

	def log(self):
		"""
		Logs some info to the SmartDashboard, and standard output.
		"""

		# Only every 1/10 second (or so) to avoid flooding networktables
		if not self.log_timer.running or not self.log_timer.hasPeriodPassed(self.log_timer_delay):
			return

		wpilib.SmartDashboard.putString('Pressure', '{0:.2f}'.format(self.get_pressure()))
		wpilib.SmartDashboard.putBoolean("Garbo?", self.is_pbot)

		self.drive.log()
		self.elevator.log()
		self.intake.log()

### MAIN ###

if __name__ == "__main__":
	wpilib.run(Robot)
