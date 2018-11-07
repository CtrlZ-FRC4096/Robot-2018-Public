"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2018
Code for robot "Atla-Z"
contact@team4096.org
"""

import wpilib

# Subsystems
import subsystems.drivetrain

# Commands
import commands.elevator
import commands.intake
import commands.drivetrain
import commands.climber

# Controls
from controls.joystick_pov import Joystick_POV
from controls.xbox_button import Xbox_Button
from controls.xbox_trigger import Xbox_Trigger
from commands.command_call import Command_Call
import controls.xbox_controller

from common.smartdashboard_update_trigger import SmartDashboard_Update_Trigger

import const

## CONSTANTS ##

# Joystick Axes
JOY_AXIS_LEFT_X			= 0
JOY_AXIS_LEFT_Y			= 1
JOY_AXIS_LEFT_SLIDER	= 3
JOY_AXIS_RIGHT_X		= 0
JOY_AXIS_RIGHT_Y		= 1
JOY_AXIS_RIGHT_SLIDER	= 3

# Invert any axes?
INVERT_JOY_LEFT_X		= True
INVERT_JOY_LEFT_Y		= True
INVERT_JOY_RIGHT_X		= True
INVERT_JOY_RIGHT_Y		= True

# Deadband
JOY_DEAD_BAND = 0.15

# Joystick Buttons
JOY_BTN_1				= 1
JOY_BTN_2				= 2
JOY_BTN_3				= 3
JOY_BTN_4				= 4
JOY_BTN_5				= 5
JOY_BTN_6				= 6
JOY_BTN_7				= 7
JOY_BTN_8				= 8
JOY_BTN_9				= 9
JOY_BTN_10				= 10
JOY_BTN_11				= 11
JOY_BTN_12				= 12
JOY_BTN_13				= 13
JOY_BTN_14				= 14

# Gamepad Axes
GP_AXIS_LEFT_X			= 0
GP_AXIS_LEFT_Y			= 1
GP_AXIS_RIGHT_X			= 2
GP_AXIS_RIGHT_Y			= 3

# Gamepad Buttons
GP_BTN_X				= 1
GP_BTN_A				= 2
GP_BTN_B				= 3
GP_BTN_Y				= 4
GP_BTN_BUMPER_L			= 5
GP_BTN_BUMPER_R			= 6
GP_BTN_TRIGGER_L		= 7
GP_BTN_TRIGGER_R		= 8
GP_BTN_BACK				= 9
GP_BTN_START			= 10
GP_BTN_STICK_L			= 11
GP_BTN_STICK_R			= 12

# Xbox Controller
# Buttons
XBOX_BTN_A				= 1
XBOX_BTN_B				= 2
XBOX_BTN_X				= 3
XBOX_BTN_Y				= 4
XBOX_BTN_LEFT_BUMPER	= 5
XBOX_BTN_RIGHT_BUMPER	= 6
XBOX_BTN_BACK 			= 7
XBOX_BTN_START 			= 8

# Axes
XBOX_AXIS_LEFT_X        = 0
XBOX_AXIS_LEFT_Y		= 1
XBOX_AXIS_RIGHT_X		= 4
XBOX_AXIS_RIGHT_Y		= 5
XBOX_BTN_LEFT_TRIGGER	= 2
XBOX_BTN_RIGHT_TRIGGER	= 3

# D-Pad
JOY_POV_NONE			= -1
JOY_POV_UP				= 0
JOY_POV_UP_RIGHT		= 45
JOY_POV_RIGHT			= 90
JOY_POV_DOWN_RIGHT		= 135
JOY_POV_DOWN			= 180
JOY_POV_DOWN_LEFT		= 225
JOY_POV_LEFT			= 270
JOY_POV_UP_LEFT			= 315

INVERT_XBOX_LEFT_X		= True
INVERT_XBOX_LEFT_Y		= False
INVERT_XBOX_RIGHT_X		= True
INVERT_XBOX_RIGHT_Y		= True


class OI:
	"""
	Operator Input - This class ties together controls and commands.
	"""
	def __init__(self, robot):

		self.robot = robot

		# Controllers
		# Xbox
		self.xbox_controller_1 = controls.xbox_controller.Xbox_Controller(0)
		self.xbox_controller_2 = controls.xbox_controller.Xbox_Controller(1)

		### COMMANDS ###

		# Drive Commands
		self.drive_command = commands.drivetrain.Drive_With_Tank_Values(
		    self.robot,
		    self._get_axis(self.xbox_controller_1, controls.xbox_controller.XBOX_AXIS_RIGHT_X, inverted = INVERT_XBOX_RIGHT_X),
		    self._get_axis(self.xbox_controller_1, controls.xbox_controller.XBOX_AXIS_LEFT_Y, inverted = INVERT_XBOX_LEFT_Y),
		)

		# Elevator
		self.elevator_command = commands.elevator.Run_Elevator(self.robot,
		    self._get_axis(self.xbox_controller_2, controls.xbox_controller.XBOX_AXIS_LEFT_Y)
		)

		# Intake
		self.intake_command = commands.intake.Run_Intake_Controller(self.robot,
		    self._get_axis(self.xbox_controller_2, controls.xbox_controller.XBOX_BTN_LEFT_TRIGGER),
		    self._get_axis(self.xbox_controller_2, controls.xbox_controller.XBOX_BTN_RIGHT_TRIGGER)
		)

		# Set default commands
		self.robot.drive.setDefaultCommand(self.drive_command)
		self.robot.elevator.setDefaultCommand(self.elevator_command)
		self.robot.intake.setDefaultCommand(self.intake_command)

		# PID tuning triggers for SD
		enc_trigger = SmartDashboard_Update_Trigger('ENC Autocorrect constant ', const.DRIVE_CORRECTION_PROPORTION_FORWARD_ENC)
		enc_trigger.whenActive(
		    Command_Call(lambda : const.update_enc_const(enc_trigger.get_key_value()))
		)

		drive_corr_trigger = SmartDashboard_Update_Trigger('Drive Correction', const.DRIVE_CORRECTION_ENABLED)
		drive_corr_trigger.whenActive(
		    Command_Call(lambda : const.update_auto_correct(drive_corr_trigger.get_key_value()))
		)

		# Autonomous Modes

		self.robot.auto_choose.addObject('Test', commands.autonomous.Test(self.robot))
		self.robot.auto_choose.addObject('Do Nothing', commands.autonomous.Do_Nothing(self.robot))
		self.robot.auto_choose.addDefault('Cross Baseline', commands.autonomous.Cross_Baseline(self.robot))
		self.robot.auto_choose.addObject('Center to Switch', commands.autonomous.Center_To_Switch_Right(self.robot))
		self.robot.auto_choose.addObject('Right to Switch', commands.autonomous.Right_To_Switch_Right(self.robot))
		self.robot.auto_choose.addObject('Right to Scale', commands.autonomous.Right_To_Scale_Right(self.robot))
		self.robot.auto_choose.addObject('Left to Switch', commands.autonomous.Left_To_Switch_Right(self.robot))
		self.robot.auto_choose.addObject('Left to Scale', commands.autonomous.Left_To_Scale_Left(self.robot))

		# Controller 1

		self.button_set_gear_low = Xbox_Button(self.xbox_controller_1, XBOX_BTN_RIGHT_BUMPER)
		self.button_set_gear_low.whenPressed(commands.drivetrain.Set_Gear_State(self.robot, const.ID_LOW_GEAR))
		self.button_set_gear_low.whenReleased(commands.drivetrain.Set_Gear_State(self.robot, const.ID_HIGH_GEAR))

		self.reset_elevator_encoder = Xbox_Button(self.xbox_controller_1, XBOX_BTN_START)
		self.reset_elevator_encoder.whenPressed(commands.elevator.Reset_Encoder(self.robot))

		self.toggle_correction = Xbox_Button(self.xbox_controller_1, XBOX_BTN_BACK)
		self.toggle_correction.whenPressed(commands.drivetrain.Toggle_Correction(self.robot))

		self.quick_turn = Xbox_Button(self.xbox_controller_1, XBOX_BTN_LEFT_BUMPER)
		self.quick_turn.whenPressed(commands.drivetrain.Set_Quick_Turn(self.robot, True))
		self.quick_turn.whenReleased(commands.drivetrain.Set_Quick_Turn(self.robot, False))

		self.climber_deploy_2 = Xbox_Button(self.xbox_controller_1, XBOX_BTN_BACK)
		self.climber_deploy_2.whenPressed(commands.climber.Climber_Deploy(self.robot))

		self.current_intake_test = Xbox_Button(self.xbox_controller_1, XBOX_BTN_Y)
		self.current_intake_test.whenPressed(commands.intake.Run_Intake_Current(self.robot, 1.0))
		self.current_intake_test.whenReleased(commands.intake.Set_Cube_State(self.robot, True))

		self.climber_up = Joystick_POV(self.xbox_controller_1, JOY_POV_UP)
		self.climber_up.whenPressed(commands.climber.Run(self.robot, -1))
		self.climber_up.whenReleased(commands.climber.Stop(self.robot))

		# Controller 2
		self.button_rotate_up = Xbox_Button(self.xbox_controller_2, XBOX_BTN_Y)
		self.button_rotate_up.whenPressed(commands.intake.Set_Rotation_Speed(self.robot, speed = -0.5))
		self.button_rotate_up.whenReleased(commands.intake.Set_Rotation_Speed(self.robot, speed = -0.1))

		self.button_rotate_down = Xbox_Button(self.xbox_controller_2, XBOX_BTN_A)
		self.button_rotate_down.whenPressed(commands.intake.Stop_Rotation(self.robot))

		self.elevator_distance = Xbox_Button(self.xbox_controller_2, XBOX_BTN_X)
		self.elevator_distance.whenPressed(commands.intake.Run_Intake_Timeout(self.robot, 0.8, 0.5))

		self.elevator_full_down = Joystick_POV(self.xbox_controller_2, JOY_POV_DOWN)
		self.elevator_full_down.whenPressed(commands.elevator.Move_Distance(self.robot, 0, setpoint = True))

		self.elevator_no_drag = Joystick_POV(self.xbox_controller_2, JOY_POV_RIGHT)
		self.elevator_no_drag.whenPressed(commands.elevator.Move_Distance(self.robot, 1, setpoint = True))

		self.elevator_switch = Joystick_POV(self.xbox_controller_2, JOY_POV_LEFT)
		self.elevator_switch.whenPressed(commands.elevator.Move_Distance(self.robot, 2, setpoint = True))

		self.elevator_scale = Joystick_POV(self.xbox_controller_2, JOY_POV_UP)
		self.elevator_scale.whenPressed(commands.elevator.Move_Distance(self.robot, 3, setpoint = True))

		#self.elevator_up_level = Xbox.Joystick_POV(self.xbox_controller_2, JOY_POV_UP)
		#self.elevator_up_level.whenPressed(commands.elevator.Move_Level(self.robot, 1))

		#self.elevator_down_level = Xbox.Joystick_POV(self.xbox_controller_2, JOY_POV_DOWN)
		#self.elevator_down_level.whenPressed(commands.elevator.Move_Level(self.robot, -1))

		self.button_intake_pistons = Xbox_Button(self.xbox_controller_2, XBOX_BTN_B)
		self.button_intake_pistons.whenPressed(commands.intake.Set_Intake_Compression(self.robot, False))
		self.button_intake_pistons.whenReleased(commands.intake.Set_Intake_Compression(self.robot, True))

	def _get_axis(self, joystick, axis, inverted = False):
		"""
		Handles inverted joy axes and dead band.
		"""
		def axis_func():
			val = joystick.getAxis(axis)

			if abs(val) < JOY_DEAD_BAND:
				val = 0

			if inverted:
				val *= -1

			return val

		return axis_func