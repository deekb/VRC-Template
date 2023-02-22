"""
Competition Code for VRC: Spin-Up (<StartYear>-<EndYear>)
Team: 3773P (Bowbots Phosphorus)
Author: Derek Baier (deekb on GitHub)
Project homepage: <Project URL>
Project archive: <Project URL>/archive/master.zip
Version: 0.0.0_rc
Contact Derek.m.baier@gmail.com for more information
"""
# <editor-fold desc="Imports and license">
from vex import *
from Constants import Color, AutonomousTask
from HelperFunctions import Drivetrain, cubic_normalize, \
    get_optical_color, CustomPID, Logging

__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up <StartYear>-<EndYear>"
__team__ = "3773P (Bowbots Phosphorus)"
__url__ = "<Project URL>"
__download_url__ = "<Project URL>/archive/master.zip"
__version__ = "0.0.0_rc"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "MIT"
# </editor-fold>


brain = Brain()


class Motors:
    """
    A class containing references to all motors and motor groups attached to the robot including motors with custom PIDs
    """
    # Motors:
    rightFrontMotor = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    rightRearMotor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
    leftFrontMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
    leftRearMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
    # Motor groups:
    leftDrivetrain = MotorGroup(leftFrontMotor, leftRearMotor)
    rightDrivetrain = MotorGroup(rightFrontMotor, rightRearMotor)
    allWheels = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)


class Sensors:
    """
    A class that contains references to all sensors attached to the robot
    """
    inertial = Inertial(Ports.PORT1)


class Controllers:
    """
    A class that contains references to the primary and secondary controllers
    """
    primary = Controller(PRIMARY)
    secondary = Controller(PARTNER)


class Globals:
    """
    Stores variables that may need to be (or ought to be able to be) accessed by any function in the program, here you can also set default/initial values for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    AUTONOMOUS_TASK = None
    STOPPING_MODE = None
    SETTINGS = (
        ("Stopping", [("Coast", COAST),
                      ("Brake", BRAKE),
                      ("Hold", HOLD)]),
        ("Autonomous", [("Skills", AutonomousTask.SKILLS),
                        ("None", AutonomousTask.DO_NOTHING),
                        ("Drivetrain Test", AutonomousTask.DRIVETRAIN_TEST)])
    )


# <editor-fold desc="Print/Clear functions">
def bprint(string) -> None:
    """
    Prints a string to the brain's screen
    :param string: the string to print to the screen
    :type string: str
    """
    brain.screen.print(string)
    brain.screen.next_row()


def bclear() -> None:
    """
    Clears the brain's screen
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


def cprint(string, controller: Controller = Controllers.primary) -> None:
    """
    Prints a string to a controller's screen
    :param controller: The controller to print to
    :type controller: Controller
    :param string: the string to print to the screen
    :type string: str
    """
    controller.screen.print(string)
    controller.screen.next_row()


def cclear(controller: Controller = Controllers.primary) -> None:
    """
    Clears the controller screen
    :param controller: The controller to clear
    :type controller: Controller
    """
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
# </editor-fold>


def update_globals() -> None:
    """
    A setup function, sets the globals to the values selected on the controller using its screen to print them
    """
    setting_index = 0
    while setting_index < len(Globals.SETTINGS):
        choice = 0
        setting_name = Globals.SETTINGS[setting_index][0]
        total_values = len(Globals.SETTINGS[setting_index][1])
        # Wait until all buttons are released
        while any((Controllers.primary.buttonLeft.pressing(),
                   Controllers.primary.buttonRight.pressing(),
                   Controllers.primary.buttonA.pressing(),
                   Controllers.primary.buttonB.pressing())):
            wait(5)
        while True:
            value_printable = Globals.SETTINGS[setting_index][1][choice][0]
            value = Globals.SETTINGS[setting_index][1][choice][1]
            cclear()
            cprint(setting_name + ": " + value_printable)
            if setting_index == 0:
                Globals.STOPPING_MODE = value
            elif setting_index == 1:
                Globals.AUTONOMOUS_TASK = value
            elif setting_index == 2:
                Globals.FLYWHEEL_AUTOSTART = value
            # Wait until a button is pressed
            while not any((Controllers.primary.buttonLeft.pressing(),
                           Controllers.primary.buttonRight.pressing(),
                           Controllers.primary.buttonA.pressing(),
                           Controllers.primary.buttonB.pressing())):
                wait(5)
            if Controllers.primary.buttonRight.pressing() and choice < total_values - 1:
                choice += 1
            elif Controllers.primary.buttonLeft.pressing() and choice > 0:
                choice -= 1
            elif setting_index > 0 and Controllers.primary.buttonB.pressing():
                setting_index -= 1
                break
            elif Controllers.primary.buttonA.pressing():
                setting_index += 1
                break
            # Wait until all buttons are released
            while any((Controllers.primary.buttonLeft.pressing(),
                       Controllers.primary.buttonRight.pressing(),
                       Controllers.primary.buttonA.pressing(),
                       Controllers.primary.buttonB.pressing())):
                pass


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    brain.screen.set_font(FontType.MONO12)
    if not Globals.SETUP_COMPLETE:
        bprint("[on_autonomous]: setup not complete, ignoring request")
        return
    global drivetrain  # Ensure we can access the custom drivetrain
    autonomous_log = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="Autonomous")

    def auton_log(string):
        """
        Send a string to the log and the brain screen
        :param string: The string to send
        """
        autonomous_log.log(string, "on_autonomous")
        bprint(string)
    brain.screen.set_font(FontType.MONO12)
    Motors.allWheels.set_stopping(BRAKE)
    drivetrain.reset()
    auton_log("Autonomous:STATUS: Start")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DO_NOTHING:
        auton_log("Autonomous:STATUS: Doing nothing")
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SKILLS:
        auton_log("Autonomous:STATUS: Running skills")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DRIVETRAIN_TEST:
        auton_log("Autonomous:STATUS: Running drivetrain test")
        drivetrain.turn_to_heading(0)
        drivetrain.move_towards_heading(0, target_speed=50, distance_mm=1000)
        drivetrain.turn_relative(180)
        drivetrain.move_towards_heading(180, target_speed=50, distance_mm=1000)
    
    auton_log("Autonomous:INFO: Cleaning up")
    Motors.allWheels.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    auton_log("Autonomous:STATUS: Exit")
    drivetrain.log.exit()
    autonomous_log.exit()


def on_driver() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    brain.screen.set_font(FontType.MONO12)
    bprint("[on_driver]: Waiting for setup thread")
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    bprint("[on_driver]: Done")
    global drivetrain
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            drivetrain.move_with_controller(Controllers.primary)


# Primary controller bindings
# Controllers.primary.buttonA.pressed()
# Secondary controller bindings
# Controllers.primary.buttonA.pressed()


# <editor-fold desc="Competition State Handlers">
def autonomous_handler() -> None:
    """
    Coordinate when to run the autonomous function using the vex competition library to read the game state.
    """
    autonomous_thread = Thread(on_autonomous)
    while competition.is_autonomous() and competition.is_enabled():
        sleep(10)
    for thread in (autonomous_thread,):
        thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function using the vex competition library to read the game state.
    """
    driver_thread = Thread(on_driver)
    while competition.is_driver_control() and competition.is_enabled():
        sleep(10)
    for thread in (driver_thread,):
        thread.stop()


# Register the competition functions
competition = Competition(driver_handler, autonomous_handler)
# </editor-fold>

if __name__ == "__main__":
    brain.screen.set_font(FontType.MONO12)
    bprint("Program: " + __title__)
    bprint("Version: " + __version__)
    bprint("Author: " + __author__)
    bprint("Team: " + __team__)
    update_globals()
    # Apply the effect of setting Globals.STOPPING_MODE during setup
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    # Initialize a new smart drivetrain from our helper functions module (Not the vex one)
    drivetrain = Drivetrain(inertial=Sensors.inertial, left_side=Motors.leftDrivetrain, right_side=Motors.rightDrivetrain, wheel_radius_mm=50, turn_aggression=0.25, correction_aggression=0.1, heading_offset_tolerance=1, motor_stall_speed=5, movement_slowdown_slope=0.2, driver_control_deadzone=0.02)
    cprint("Calibrating Gyro...")
    Sensors.inertial.calibrate()
    while Sensors.inertial.is_calibrating():
        pass
    cclear()
    Globals.SETUP_COMPLETE = True
    cprint("Setup complete")
    bprint("Setup complete")
    Controllers.primary.rumble(".")
    Controllers.secondary.rumble(".")
