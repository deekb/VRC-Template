"""
All custom constants needed for our programs
"""
from vex import *


# SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc
# it should be set between 0.00 and 3.00 for optimal performance
SPEED_CURVE_LINEARITY = 0.35
AUTONOMOUS_VERBOSITY = 0  # 0-2, 0: nothing, 1: logging, 2: logging & printing
BACKGROUND_IMAGE = None  # Set this to the name of a bitmap (BMP) image or None to disable


class Color:
    """
    Any colors needed in the program
    """
    RED = 0
    YELLOW = 1
    BLUE = 2


class AutonomousTask:
    """
    All autonomous tasks
    """
    SKILLS = "Skills"
    DO_NOTHING = "Nothing"
    DRIVETRAIN_TEST = "Drivetrain Test"


class DriverControlStyle:
    """
    Drivetrain control style
    """
    TANK = 0
    ARCADE = 1


class DrivetrainType:
    """
    Type of drivetrain
    """
    TANK = 0
    X_DRIVE = 1
