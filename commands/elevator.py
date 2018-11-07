"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.command.commandgroup import Command

import subsystems.elevator

import const
import ctre

class Run_Elevator(Command):

	def __init__(self, robot, speed):

		super().__init__()

		self.robot = robot
		self.speed = speed

		self.requires(self.robot.elevator)
		self.setInterruptible(True)
		
	def initialize(self):
		self.robot.elevator.release_brake()

	def execute(self):
		

		if callable(self.speed):
			speed = self.speed()
			if speed < 0:
				speed = speed * -speed
			else:
				speed = speed * speed
		
		#if self.robot.elevator.limit_switch.get():
		#	self.robot.elevator.talon.setSelectedSensorPosition(0, 0, 10)
		
		if speed == 0 and not self.robot.elevator.talon.getControlMode() == ctre.WPI_TalonSRX.ControlMode.MotionMagic:
			self.robot.elevator.run_elevator(0)	
			self.robot.elevator.set_brake()
		else:
			self.robot.elevator.release_brake()
			self.robot.elevator.run_elevator(speed)

	def isFinished(self):
		return False

	def end(self):
		self.robot.elevator.stop_elevator()
		self.robot.elevator.set_brake()

	def interrupted(self):
		self.end()


class Stop_Elevator(Command):
	def __init__(self, robot):

		super().__init__()

		self.robot = robot

		self.requires(self.robot.elevator)
		self.setInterruptible(True)

	def execute(self):
		self.robot.elevator.run_elevator(0)
		self.robot.elevator.set_brake()

	def isFinished(self):
		return True		

class Reset_Encoder(Command):
	def __init__(self, robot):

		super().__init__()

		self.robot = robot

		self.requires(self.robot.elevator)
		self.setInterruptible(True)

	def execute(self):
		self.robot.elevator.talon.setSelectedSensorPosition(0, 0, self.robot.elevator.kTimeoutMs)

	def isFinished(self):
		return True		

class Set_Brake(Command):
	def __init__(self, robot):

		super().__init__()

		self.robot = robot
		
		self.requires(self.robot.elevator)
		self.setInterruptible(True)

	def execute(self):
		self.robot.elevator.set_brake()

	def isFinished(self):
		return True


class Release_Brake(Command):

	def __init__(self, robot):

		super().__init__()

		self.robot = robot

		self.requires(self.robot.elevator)
		self.setInterruptible(True)

	def execute(self):
		self.robot.elevator.release_brake()

	def isFinished(self):
		return True


class Move_Distance(Command):
	def __init__ (self, robot, distance, max_speed = None, setpoint = False, relative = False, timeout = 10 ):
		super().__init__()

		self.robot = robot

		self.requires(self.robot.elevator)
		
		self.distance = distance
		self.index = distance
		self.physical_distance = 0
		self.setpoint = setpoint
		self.relative = relative
		
		self.setTimeout( timeout )

	def initialize(self):
		if self.setpoint:
			if self.relative:
				if not (self.robot.elevator.position + self.index == len(const.ELEVATOR_POSITION_DICT)) and (self.robot.elevator.position + self.index >= 0):
					self.robot.elevator.position += self.index
			else:
				self.robot.elevator.position = self.index
			self.physical_distance = -const.ELEVATOR_POSITION_DICT[self.robot.elevator.position] * const.ELEVATOR_TICKS_PER_FOOT
			print("Setpoint!", self.robot.elevator.position)
		else:
			self.physical_distance = -self.distance * const.ELEVATOR_TICKS_PER_FOOT
		
		self.robot.elevator.release_brake()		
		
	def execute(self):
		self.robot.elevator.go_magic_distance(self.physical_distance)		
		print( 'going to magic distance:', self.physical_distance)
		
	def isFinished(self):
		enc_position = self.robot.elevator.talon.getQuadraturePosition()
		print( 'enc:', enc_position)
		
		val = abs(abs(self.physical_distance) - abs(enc_position))
		#print( 'elevator val =', val )
		
		if val <= 5000 or self.isTimedOut():
			#print( 'Elevator Move finished' )
			return True
		else:
			return False
		
	def end(self):
		#print( 'Elevator Move ending' )
		self.robot.elevator.set_brake()
		self.robot.elevator.talon.set(0)
		self.robot.elevator.victor_spx.set(0)
		
	def interrupted(self):
		self.end()