import math
import pathfinder as pf
import os.path
import pickle
import wpilib

import const

pickle_file_name = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')

if wpilib.RobotBase.isSimulation():
    def gen_trajectory(points, spline_type, backwards = False):
        if backwards:
            points = list(reversed(points))
        """Returns a tuple of path information (TrajectoryInfo) and trajectory (Segment)"""
        return pf.generate(points, spline_type, pf.SAMPLES_HIGH,
                           dt=0.05, # 50ms
                           max_velocity=const.DRIVE_MAX_SPEED,
                           max_acceleration=const.DRIVE_MAX_ACCEL,
                           max_jerk=const.DRIVE_MAX_JERK)


    # Cross Baseline

    cb_1_points = [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(12, 0, 0),
    ]

    cb_1_pathinfo, cb_1_trajectory = gen_trajectory(cb_1_points, pf.FIT_HERMITE_CUBIC)

    cb_1_modifier = pf.modifiers.TankModifier(cb_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)

    # Center to Switch Right

    cswr_1_points = [
                    pf.Waypoint(0, 13, 0),
                    pf.Waypoint(11, 8, 0)
    ]

    cswr_1_pathinfo, cswr_1_trajectory = gen_trajectory(cswr_1_points, pf.FIT_HERMITE_CUBIC)

    cswr_1_modifier = pf.modifiers.TankModifier(cswr_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)

    # Center to Switch Left

    cswl_1_points = [
                    pf.Waypoint(0, 13, 0),
                    pf.Waypoint(11, 18, 0),
    ]

    cswl_1_pathinfo, cswl_1_trajectory = gen_trajectory(cswl_1_points, pf.FIT_HERMITE_CUBIC)

    cswl_1_modifier = pf.modifiers.TankModifier(cswl_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)

    # rswr = Right to Right Switch

    # Part 1: Drive to Switch
    rswr_1_points = [
            pf.Waypoint(0, 3.5, 0),
                pf.Waypoint(8, 3.5, 0),
                pf.Waypoint(14, 7, math.radians(90)),
        ]

    rswr_1_pathinfo, rswr_1_trajectory = gen_trajectory(rswr_1_points, pf.FIT_HERMITE_CUBIC)

    rswr_1_modifier = pf.modifiers.TankModifier(rswr_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # rscr = Right to Right Scale

    # Part 1: Drive to Scale
    rscr_1_points = [
                pf.Waypoint(0, 3.5, 0),
                pf.Waypoint(14, 2, 0),
                pf.Waypoint(25, 5.5, math.radians(45)),
        ]

    rscr_2_points = [
                pf.Waypoint(25, 5.5, math.radians(45)),
                pf.Waypoint(27, 7.5, math.radians(30))
    ]

    rscr_3_points = [
                pf.Waypoint(27, 7.5, math.radians(30)),
                pf.Waypoint(14, 4, 0)
    ]

    rscr_1_pathinfo, rscr_1_trajectory = gen_trajectory(rscr_1_points, pf.FIT_HERMITE_CUBIC)
    rscr_2_pathinfo, rscr_2_trajectory = gen_trajectory(rscr_2_points, pf.FIT_HERMITE_CUBIC)
    rscr_3_pathinfo, rscr_3_trajectory = gen_trajectory(rscr_3_points, pf.FIT_HERMITE_CUBIC, backwards = True)

    rscr_1_modifier = pf.modifiers.TankModifier(rscr_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)
    rscr_2_modifier = pf.modifiers.TankModifier(rscr_2_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)
    rscr_3_modifier = pf.modifiers.TankModifier(rscr_3_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # rswl = Right to Left Switch

    # Part 1: Drive to Back of Switch

    rswl_1_points = [
            pf.Waypoint(0, 3.5, 0),
                pf.Waypoint(14, 3.5, 0),
                pf.Waypoint(20, 8, math.radians(90)),
                pf.Waypoint(20, 14, math.radians(90)),
                pf.Waypoint(17.5, 18, math.radians(180)),
        ]

    rswl_1_pathinfo, rswl_1_trajectory = gen_trajectory(rswl_1_points, pf.FIT_HERMITE_CUBIC)

    rswl_1_modifier = pf.modifiers.TankModifier(rswl_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # rscl = Right to Left Scale

    # Part 1: Drive to Scale
    rscl_1_points = [
            pf.Waypoint(0, 3.5, 0),
                pf.Waypoint(20, 3.5, 0),
                pf.Waypoint(22, 8, math.radians(90)),
                pf.Waypoint(20, 16, math.radians(90)),
                pf.Waypoint(24, 19.5, 0),
        ]

    rscl_1_pathinfo, rscl_1_trajectory = gen_trajectory(rscl_1_points, pf.FIT_HERMITE_CUBIC)

    rscl_1_modifier = pf.modifiers.TankModifier(rscl_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # lswl = Left to Left Switch

    # Part 1: Drive to Switch
    lswl_1_points = [
            pf.Waypoint(0, 23.5, 0),
                pf.Waypoint(8, 23.5, 0),
                pf.Waypoint(15, 20, math.radians(-90)),
        ]

    lswl_1_pathinfo, lswl_1_trajectory = gen_trajectory(lswl_1_points, pf.FIT_HERMITE_CUBIC)

    lswl_1_modifier = pf.modifiers.TankModifier(lswl_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # lswr = Left to Right Switch

    # Part 1: Drive to Switch

    lswr_1_points = [
            pf.Waypoint(0, 23.5, 0),
                pf.Waypoint(14, 23.5, 0),
                pf.Waypoint(20, 19, math.radians(90)),
                pf.Waypoint(20, 13, math.radians(90)),
                pf.Waypoint(17.5, 9, math.radians(180)),
        ]

    lswr_1_pathinfo, lswr_1_trajectory = gen_trajectory(lswr_1_points, pf.FIT_HERMITE_CUBIC)

    lswr_1_modifier = pf.modifiers.TankModifier(lswr_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)


    # lscl = Left to Left Scale           #CHANGEEEEEEEEE

    # Part 1: Drive to Scale
    lscl_1_points = [
                pf.Waypoint(0, 23.5, 0),
                pf.Waypoint(14, 25, 0),
                pf.Waypoint(24.5, 23, math.radians(-45)),
    ]

    lscl_2_points = [
                pf.Waypoint(24.5, 23, math.radians(-45)),
                pf.Waypoint(25.5, 22.5, math.radians(-35))
    ]

    lscl_3_points = [
                pf.Waypoint(25.5, 22.5, math.radians(-35)),
                pf.Waypoint(14, 25, 0)
    ]

    lscl_1_pathinfo, lscl_1_trajectory = gen_trajectory(lscl_1_points, pf.FIT_HERMITE_CUBIC)
    lscl_2_pathinfo, lscl_2_trajectory = gen_trajectory(lscl_2_points, pf.FIT_HERMITE_CUBIC)
    lscl_3_pathinfo, lscl_3_trajectory = gen_trajectory(lscl_3_points, pf.FIT_HERMITE_CUBIC, backwards = True)

    lscl_1_modifier = pf.modifiers.TankModifier(lscl_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)
    lscl_2_modifier = pf.modifiers.TankModifier(lscl_2_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)
    lscl_3_modifier = pf.modifiers.TankModifier(lscl_3_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)

    # lscr = Left to Right Scale

    # Part 1: Drive to Scale
    lscr_1_points = [
                pf.Waypoint(0, 23.5, 0),
                pf.Waypoint(14, 24, math.radians(-15)),
                pf.Waypoint(18, 22, math.radians(-45)),
                pf.Waypoint(19.50, 19, math.radians(-60)),
                pf.Waypoint(19.75, 14, math.radians(-90)),
                pf.Waypoint(20.50, 8, math.radians(-60)),
    ]

    lscr_2_points = [
        pf.Waypoint(20.50, 8, math.radians(-60)),
        pf.Waypoint(24, 5, 0),
    ]

    lscr_1_pathinfo, lscr_1_trajectory = gen_trajectory(lscr_1_points, pf.FIT_HERMITE_CUBIC)
    lscr_2_pathinfo, lscr_2_trajectory = gen_trajectory(lscr_2_points, pf.FIT_HERMITE_CUBIC)

    lscr_1_modifier = pf.modifiers.TankModifier(lscr_1_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)
    lscr_2_modifier = pf.modifiers.TankModifier(lscr_2_trajectory).modify(const.DRIVE_WHEELBASE_WIDTH)

    # Dictionary of all mode tuples to be pickled

    mode_dict = {
        "CB": cb_1_modifier,
        "CSWL": cswl_1_modifier,
        "CSWR": cswr_1_modifier,
        "RSWR": rswr_1_modifier,
        "RSWL": rswl_1_modifier,
        "RSCR": (rscr_1_modifier, rscr_2_modifier, rscr_3_modifier),
        "RSCL": rscl_1_modifier,
        "LSWL": lswl_1_modifier,
        "LSWR": lswr_1_modifier,
        "LSCL": (lscl_1_modifier, lscl_2_modifier, lscl_3_modifier),
        "LSCR": (lscr_1_modifier, lscr_2_modifier),
    }

    # Pickle our dictionary
    with open(pickle_file_name, 'wb') as fp:
        pickle.dump(mode_dict, fp)

else:
    # Open the jar of pickles
    with open(pickle_file_name, 'rb') as fp:
        mode_dict = pickle.load(fp)