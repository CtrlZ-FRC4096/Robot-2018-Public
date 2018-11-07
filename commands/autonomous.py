"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

from wpilib.command.commandgroup import CommandGroup
from wpilib.command import WaitCommand

import const
import commands.drivetrain
import paths

class Raise_Elevator_Delay(CommandGroup):
	'''
	Raises the elevator after a period of 3 seconds
	'''
	def __init__(self, robot):
		super().__init__()

		self.robot = robot

		self.addSequential(WaitCommand(3))
		self.addSequential(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))		# Raises the elevator


class Do_Nothing(CommandGroup):
	'''
	This autonomous mode does... wait for it... nothing.
	'''
	def __init__(self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "DN"

		self.addSequential(commands.drivetrain.Stop(self.robot))

class Cross_Baseline(CommandGroup):
	'''
	Crosses the baseline
	'''
	def __init__(self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "CB"

		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 4))

class Center_To_Switch_Right(CommandGroup):
	'''
	Drives from the left starting position to the left side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "CSWR"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 4))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Center_To_Switch_Left(CommandGroup):
	'''
	Drives from the left starting position to the left side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "CSWL"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 4))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Left_To_Scale_Left(CommandGroup):
	'''
	Drives from the left starting position to the left side of the scale
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "LSCL"

		#self.addParallel(commands.elevator.Move_Distance(self.robot, 1, setpoint = True))
		self.addParallel(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][0], 10))
		self.addSequential(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][1], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -1.0, 0.5))
		self.addSequential(commands.drivetrain.Follow_Path_Backwards(self.robot, paths.mode_dict[self.string_name][2], 10))
		self.addSequential(commands.elevator.Move_Distance(self.robot, 0, setpoint = True))

class Left_To_Scale_Right(CommandGroup):
	'''
	Drives from the left starting position to the right side of the scale
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "LSCR"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 1, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][0], 10))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][1], 10))
		#self.addSequential(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))
		#self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Left_To_Switch_Left(CommandGroup):
	'''
	Drives from the left starting position to the left side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "LSWL"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Left_To_Switch_Right(CommandGroup):
	'''
	Drives from the left starting position to the right side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "LSWR"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Right_To_Scale_Right(CommandGroup):
	'''
	Drives from the right starting position to the right side of the scale
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "RSCR"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 1, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][0], 10))
		self.addSequential(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name][1], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -1.0, 0.5))
		#self.addParallel(commands.elevator.Move_Distance(self.robot, 0, setpoint = True))
		#self.addSequential(commands.drivetrain.Follow_Path_Backwards(self.robot, paths.mode_dict[self.string_name][2], 10))

class Right_To_Scale_Left(CommandGroup):
	'''
	Drives from the right starting position to the left side of the scale
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "RSCL"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 1, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 10))
		self.addSequential(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Right_To_Switch_Right(CommandGroup):
	'''
	Drives from the right starting position to the right side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "RSWR"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))

class Right_To_Switch_Left(CommandGroup):
	'''
	Drives from the right starting position to the left side of the switch
	'''
	def __init__ (self, robot):
		super().__init__()

		self.robot = robot
		self.string_name = "RSWL"

		self.addParallel(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))
		self.addSequential(commands.drivetrain.Follow_Path(self.robot, paths.mode_dict[self.string_name], 10))
		self.addSequential(commands.intake.Run_Intake_Timeout(self.robot, -0.75, 0.5))