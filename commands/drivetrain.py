"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.command.commandgroup import Command, CommandGroup
from wpilib.command.waitcommand import WaitCommand

import pathfinder as pf
from pathfinder.followers import EncoderFollower

import const
import subsystems.drivetrain

class Drive_With_Tank_Values(Command):
	def __init__(self, robot, get_r, get_y):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param get_r: Used to get the x angle, function that determines y direction
		:param get_y: Used to get the y angle, function that determines the y value and direction
		rotation and direction of rotation. Z value must be given if it separate from the joystick.
		'''
		super().__init__()

		self.robot = robot

		self.requires(self.robot.drive)
		self.setInterruptible(True)

		self.get_r = get_r
		self.get_y = get_y

	def execute(self):
		if not const.DEMO_MODE and abs(self.robot.elevator.talon.getQuadraturePosition()) > abs(2.5 * const.ELEVATOR_TICKS_PER_FOOT):
			if self.robot.drive.gear_state == const.ID_LOW_GEAR:
				self.robot.drive.joymult = 0.7
				self.robot.drive.rotmult = 0.9
			else:
				self.robot.drive.joymult = 0.6
				self.robot.drive.rotmult = 0.8
		else:
			self.robot.drive.joymult = 1.0
			self.robot.drive.rotmult = 1.0
		
		if callable(self.get_y):
			y = self.get_y()
			if y < 0:
				y = y * -y
			else:
				y = y * y
		else:
			y = self.get_y

		# If in tank mode, still use mecanum drive code but force X/strafe to zero.
		r = self.get_r() if callable(self.get_r) else self.get_r
		
		y *= self.robot.drive.joymult
		r *= self.robot.drive.rotmult
		
		# If demo mode, reduce speed by some amount
		if const.DEMO_MODE:
			r *= const.DEMO_MODE_DRIVETRAIN_SPEED_MULTIPLIER_R
			y *= const.DEMO_MODE_DRIVETRAIN_SPEED_MULTIPLIER_Y
			
		self.robot.drive.drive_with_tank_values(r * const.DRIVE_ROTATION_MULTIPLIER, y)

	def isFinished(self):
		return False

	def end(self):
		self.robot.drive.stop()

	def interrupted(self):
		self.end()


class Drive_Distance(Command):
	def __init__ (self, robot, distance, max_speed = None):
		super().__init__()

		self.robot = robot
		self.distance = distance
		self.driving_straight = (distance > 0) # true if distance is positive
		self.distance_remaining = distance
		self.distance_traveled = 0.0
		self.setInterruptible(False)

		self.requires(self.robot.drive)
		self.setTimeout(6)
		
	def initialize(self):
		self.robot.drive_encoder_left.reset()
		self.robot.drive_encoder_right.reset()
		self.robot.drive.drive_distance_pid.enable()
		print("Setpoint:", self.distance)
		self.robot.drive.drive_distance_pid.setSetpoint(self.distance)

	def isFinished(self):
		on_target = self.robot.drive.drive_distance_pid.onTarget()
		timed_out = self.isTimedOut()

		if timed_out:
			print('Drive Distance Timed out!')
		if on_target:
			self.distance_traveled = self.robot.drive.get_drive_distance_pid_input()
			print('Drive Distance On Target! {0:.2f}'.format(self.distance_traveled))
		
		finished = on_target # or self.isTimedOut()

		return finished

	def end(self):
		#print("Ending Drive Distance")
		self.robot.drive.drive_distance_pid.disable()

	def interrupted(self):
		self.end()
		

class Drive_Distance_And_Turn(Command):
	"""
	Uses both drive forward and rotate PIDControllers to move and rotate at the
	same time, resulting in a curve.
	"""
	def __init__ (self, robot, distance, angle, max_speed = None):
		super().__init__()

		self.robot = robot
		self.driving_straight = distance > 0 # true if distance is positive
		self.distance_remaining = distance
		self.distance_traveled = 0.0		
		self._old_drive_correction_value = None

		self.requires(self.robot.drive)
		self.setInterruptible(True)
		self.setTimeout(6)
		
	def initialize(self):
		# Temporarily disable drive correction, if enabled
		self._old_drive_correction_value = const.DRIVE_CORRECTION_ENABLED
		const.DRIVE_CORRECTION_ENABLED = False

		self.robot.drive_encoder_left.reset()
		self.robot.drive_encoder_right.reset()
		self.robot.drive.drive_distance_pid.enable()
		self.robot.drive.drive_distance_pid.setSetpoint(self.distance)
		print('setpoints: {0}, {1}'.format(self.distance, self.angle))

		self.robot.gyro.reset()
		self.robot.drive.rotate_pid.enable()
		self.robot.drive.rotate_pid.setSetpoint(self.angle)

	def isFinished(self):	
		if self.robot.drive.drive_distance_pid.isEnabled():
			on_target_drive = self.robot.drive.drive_distance_pid.onTarget()
		else:
			on_target_drive = True

		if on_target_drive:
			self.robot.drive.drive_distance_pid.disable()

		if self.robot.drive.rotate_pid.isEnabled():
			on_target_rotate = self.robot.drive.rotate_pid.onTarget()
		else:
			on_target_rotate = True

		if on_target_rotate:
			self.robot.drive.rotate_pid.disable()

		timed_out = self.isTimedOut()

		if timed_out:
			print('Drive Distance And Turn timed out!')
			return True

		#print('Drive: {0}, Rotate: {1}'.format(on_target_drive, on_target_rotate))

		if on_target_drive and on_target_rotate:
			self.distance_traveled = self.robot.drive.get_drive_distance_pid_input()
			print('Drive Distance And Turn On Target! {0:.2f}, {1:.2f}'.format(self.distance_traveled, self.robot.gyro.getAngle()))

			# Turn correction back on, if it was enabled
			const.DRIVE_CORRECTION_ENABLED = self._old_drive_correction_value

		finished = on_target_drive and on_target_rotate # or self.isTimedOut()

		return finished

	def end(self):
		self.robot.drive.drive_distance_pid.disable()
		self.robot.drive.rotate_pid.disable()

	def interrupted(self):
		self.end()


class Rotate_To_Angle(Command):
	def __init__(self, robot, angle):
		super().__init__()

		self.robot = robot
		self.angle = angle

		self.requires(self.robot.drive)
		self.setInterruptible(True)
		self.setTimeout(5)		
		
	def initialize(self):
		self.robot.gyro.reset()
		#sleep(.25)
		#print('gyro start = {0:.2f}'.format(self.robot.gyro.getAngle()))
		self.robot.drive.rotate_pid.enable()
		self.robot.drive.rotate_pid.setSetpoint(self.angle)

	def isFinished(self):
		on_target = self.robot.drive.rotate_pid.onTarget()
		timed_out = self.isTimedOut()

		if timed_out:
			print('Rotate To Angle Timed out!')
			return True
		if on_target:
			print('Rotate On Target! {0:.2f}'.format(self.robot.gyro.getAngle()))

		finished = on_target # or self.isTimedOut()

		return finished

	def end(self):
		self.robot.drive.rotate_pid.disable()

	def interrupted(self):
		self.end()

class Follow_Path(Command):
	def __init__ (self, robot, modifier, timeout):
		super().__init__()

		self.robot = robot
		self.modifier = modifier
		
		self.setInterruptible(False)

		self.requires(self.robot.drive)
		self.setTimeout(timeout)
	
	def initialize(self):
		
		self.robot.left_enc_follower = EncoderFollower(self.modifier.getLeftTrajectory())
		self.robot.right_enc_follower = EncoderFollower(self.modifier.getRightTrajectory())

		self.robot.left_enc_follower.reset()
		self.robot.right_enc_follower.reset()

		self.robot.drive_encoder_left.reset()
		self.robot.drive_encoder_right.reset()

		self.robot.drive_encoder_left.setReverseDirection(const.DRIVE_ENCODER_LEFT_REVERSED)
		self.robot.drive_encoder_right.setReverseDirection(const.DRIVE_ENCODER_RIGHT_REVERSED)

		self.robot.left_enc_follower.configureEncoder(self.robot.drive_encoder_left.get(), 256, 6.25 / 12)
		self.robot.right_enc_follower.configureEncoder(self.robot.drive_encoder_right.get(), 256, 6.25 / 12)
	
		self.robot.left_enc_follower.configurePIDVA(const.DRIVE_kP, 0.0, const.DRIVE_kD, 1 / const.DRIVE_MAX_SPEED, 0)
		self.robot.right_enc_follower.configurePIDVA(const.DRIVE_kP, 0.0, const.DRIVE_kD, 1 / const.DRIVE_MAX_SPEED, 0)				
		
	def execute(self):
		l = self.robot.left_enc_follower.calculate(self.robot.drive_encoder_left.get())
		r = self.robot.right_enc_follower.calculate(self.robot.drive_encoder_right.get())
		
		gyro_heading = -self.robot.gyro.getAngle()    # Assuming the gyro is giving a value in degrees
		desired_heading = pf.r2d(self.robot.left_enc_follower.getHeading())   # Should also be in degrees

		# This is a poor man's P controller
		angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
		turn = 5 * (-1.0 / 180.0) * angleDifference
		
		#print(turn)
			
		l += turn
		r -= turn
	
		# -1 is forward, so invert both values
		self.robot.drive.drive.tankDrive(-l, -r)

	def end(self):
		self.robot.drive.drive.tankDrive(0, 0)
	
	def interrupted(self):
		self.end()
	
	def isFinished(self):
		return self.robot.left_enc_follower.isFinished() and self.robot.right_enc_follower.isFinished()

class Follow_Path_Backwards(Command):
	def __init__ (self, robot, modifier, timeout):
		super().__init__()

		self.robot = robot
		self.modifier = modifier
		
		self.setInterruptible(False)

		self.requires(self.robot.drive)
		self.setTimeout(timeout)

	def initialize(self):

		print("Backwards")
		
		self.robot.left_enc_follower = EncoderFollower(self.modifier.getLeftTrajectory())
		self.robot.right_enc_follower = EncoderFollower(self.modifier.getRightTrajectory())

		self.robot.left_enc_follower.reset()
		self.robot.right_enc_follower.reset()

		self.robot.drive_encoder_left.reset()
		self.robot.drive_encoder_right.reset()

		self.robot.drive_encoder_left.setReverseDirection(not const.DRIVE_ENCODER_LEFT_REVERSED)
		self.robot.drive_encoder_right.setReverseDirection(not const.DRIVE_ENCODER_RIGHT_REVERSED)

		self.robot.left_enc_follower.configureEncoder(self.robot.drive_encoder_left.get(), 256, 6.25 / 12)
		self.robot.right_enc_follower.configureEncoder(self.robot.drive_encoder_right.get(), 256, 6.25 / 12)

		self.robot.left_enc_follower.configurePIDVA(const.DRIVE_kP, 0.0, const.DRIVE_kD, 1 / const.DRIVE_MAX_SPEED, 0.0)
		self.robot.right_enc_follower.configurePIDVA(const.DRIVE_kP, 0.0, const.DRIVE_kD, 1 / const.DRIVE_MAX_SPEED, 0.0)

	def execute(self):
		l = self.robot.left_enc_follower.calculate(self.robot.drive_encoder_left.get())
		r = self.robot.right_enc_follower.calculate(self.robot.drive_encoder_right.get())

		gyro_heading = self.robot.gyro.getAngle()    # Assuming the gyro is giving a value in degrees
		desired_heading = -pf.r2d(self.robot.left_enc_follower.getHeading())   # Should also be in degrees

		# This is a poor man's P controller
		self.angleDifference = pf.boundHalfDegrees(desired_heading - gyro_heading)
		turn = 5 * (1.0 / 180.0) * self.angleDifference

		#print(desired_heading)		
		
		l += turn
		r -= turn

		# -1 is forward, so invert both values
		self.robot.drive.drive.tankDrive(r, l)

	def end(self):
		self.robot.drive.drive.tankDrive(0, 0)

	def interrupted(self):
		self.end()

	def isFinished(self):
		return self.robot.left_enc_follower.isFinished() and self.robot.right_enc_follower.isFinished() and (abs(self.angleDifference) < 10)
		
class Stop(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot

		self.requires(self.robot.drive)
		self.setInterruptible(True)

	def execute(self):
		print( 'Stop' )
		self.robot.drive.drive_with_tank_values(0, 0, 0)

	def isFinished(self):
		return True

class Toggle_Gear_State(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot

		self.requires(self.robot.drive)
		self.setInterruptible(True)
	
	def execute(self):
		self.robot.drive.toggle_gear_state()
		
	def isFinished(self):
		return True

class Set_Gear_State(Command):
	def __init__(self, robot, gear_id):
		super().__init__()

		self.robot = robot		
		self.gear_id = gear_id

		self.requires(self.robot.drive)
		self.setInterruptible(True)
	
	def execute(self):
		self.robot.drive.set_gear_state(self.gear_id)
		
	def isFinished(self):
		return True

class Toggle_Correction(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot

		self.requires(self.robot.drive)
		self.setInterruptible(True)

	def execute(self):
		const.DRIVE_CORRECTION_ENABLED = not const.DRIVE_CORRECTION_ENABLED

	def isFinished(self):
		return True

class Reset_Gyro(Command):
	def __init__(self, robot):
		super().__init__()

		self.robot = robot		
		
		self.requires(self.robot.drive)
		self.setInterruptible(True)

	def execute(self):
		self.robot.gyro.reset()

	def isFinished(self):
		return True
		
class Drive_Distance_Time(Command):
	def __init__ (self, robot, time, speed):
		super().__init__()

		self.robot = robot
		self.speed = speed

		self.requires(self.robot.drive)		
		self.setInterruptible(True)
		self.setTimeout(time)

	def execute(self):
		self.robot.drive.drive_with_tank_values(0, self.speed)

	def isFinished(self):
		timed_out = self.isTimedOut()
		
		return timed_out
	
class Set_Quick_Turn(Command):
	def __init__ (self, robot, quick_turn):
		super().__init__()

		self.robot = robot
		self.quick_turn = quick_turn

		self.requires(self.robot.drive)
		self.setInterruptible(False)

	def execute(self):
		self.robot.drive.quick_turn = self.quick_turn

	def isFinished(self):
		return True