"""
All custom constants needed for our programs
"""
from vex import *


class Color:
    """
    Any colors needed in the program
    """
    RED = 0
    YELLOW = 1
    BLUE = 2


class AutonomousTask:
    """
    All the possible autonomous tasks
    """
    SKILLS = 0
    DO_NOTHING = 1
    DRIVETRAIN_TEST = 2


class DrivetrainControlStyle:
    """
    Drivetrain control type
    """
    TANK = 0
    ARCADE = 1
