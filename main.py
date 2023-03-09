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

from Constants import *
from HelperFunctions import Logging
from BetterDrivetrain import Drivetrain

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
    allWheels = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)  # A reference to all wheels so that we can stop the drivetrain the robot in one command
    allMotors = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)  # A reference to all motors so that we can stop everything on the robot in one command


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
    Stores variables that may need to be (or ought to be able to be) modified by any function in the program, here you can also set default/initial values for said variables
    """
    AUTONOMOUS_THREADS = []
    DRIVER_CONTROL_THREADS = []
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False  # This can be used for autonomous functions during driver control, for example if we want to turn towards the goal
    AUTONOMOUS_TASK = None
    STOPPING_MODE = None
    DRIVE_CONTROLLER = Controllers.primary
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


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    brain.screen.set_font(FontType.MONO12)
    if not Globals.SETUP_COMPLETE:
        bprint("[on_autonomous]: setup not complete, ignoring request")
        return
    autonomous_log = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="Autonomous")  # Start a new log for this autonomous

    def auton_log(string):
        """
        Send a string to the log and the brain screen
        :param string: The string to send
        """
        if AUTONOMOUS_VERBOSITY >= 1:
            autonomous_log.log(string, "on_autonomous")
            if AUTONOMOUS_VERBOSITY >= 2:
                bprint(string)

    brain.screen.set_font(FontType.MONO12)
    Motors.allWheels.set_stopping(BRAKE)
    drivetrain.reset()
    auton_log("Autonomous:STATUS: Start")
    auton_log("Autonomous:STATUS: Running predefined autonomous routine \"" + Globals.AUTONOMOUS_TASK + "\"")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DO_NOTHING:
        pass
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SKILLS:
        pass
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DRIVETRAIN_TEST:
        drivetrain.turn_to_heading(0)
        drivetrain.move_towards_heading(0, target_speed=50, distance_mm=1000)
        drivetrain.turn_to_heading(180)
        drivetrain.move_towards_heading(180, target_speed=50, distance_mm=1000)
        drivetrain.turn_to_heading(180)
        drivetrain.move_towards_heading(180, target_speed=-50, distance_mm=1000)
    auton_log("Autonomous:STATUS: Cleaning up")
    Motors.allMotors.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    auton_log("Autonomous:STATUS: Exit")
    autonomous_log.exit()


def on_driver() -> None:
    """
    This is the function designated to run when the driver control portion of the program is triggered
    """
    # Wait for setup to be complete
    brain.screen.set_font(FontType.MONO12)
    bprint("[on_driver]: Waiting for setup")
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    bprint("[on_driver]: Done")
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            drivetrain.move_with_controller(Globals.DRIVE_CONTROLLER)
        else:
            wait(10)


# <editor-fold desc="Competition State Handlers">
def autonomous_handler() -> None:
    """
    Coordinate when to run the autonomous function(s) using the vex competition library to read the game state.
    """
    for _function in (on_autonomous,):
        Globals.AUTONOMOUS_THREADS.append(Thread(target=_function))
    bprint("Started autonomous")
    while competition.is_autonomous() and competition.is_enabled():
        wait(10)
    for thread in Globals.AUTONOMOUS_THREADS:
        thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function(s) using the vex competition library to read the game state.
    """
    for _function in (on_driver,):
        Globals.DRIVER_CONTROL_THREADS.append(Thread(target=_function))
    bprint("Started driver control")
    while competition.is_driver_control() and competition.is_enabled():
        wait(10)
    for thread in Globals.DRIVER_CONTROL_THREADS:
        thread.stop()


# Register the competition handlers
competition = Competition(driver_handler, autonomous_handler)


# </editor-fold>


def update_globals() -> None:
    """
    A setup function, sets the globals to the values selected on the main controller using its screen to print them
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


if __name__ == "__main__":
    brain.screen.set_font(FontType.MONO12)
    if BACKGROUND_IMAGE:
        brain.screen.draw_image_from_file(str(BACKGROUND_IMAGE), 0, 0)
    bprint("Program: " + __title__)
    bprint("Version: " + __version__)
    bprint("Author: " + __author__)
    bprint("Team: " + __team__)
    while True:
        update_globals()
        # Apply the effect of setting Globals.STOPPING_MODE during setup
        Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
        cprint("Please confirm", Controllers.secondary)
        cprint("Auton: " + str(Globals.AUTONOMOUS_TASK), Controllers.secondary)
        while not any((Controllers.secondary.buttonA.pressing(), Controllers.secondary.buttonB.pressing())):
            wait(5)
        if Controllers.secondary.buttonA.pressing():
            break
        else:
            Controllers.primary.rumble("-")
            cclear()
    # Initialize a new smart drivetrain from our helper functions module (Not the vex one)
    drivetrain = Drivetrain(inertial=Sensors.inertial, left_side_motors=(Motors.leftFrontMotor, Motors.leftRearMotor),
                            right_side_motors=(Motors.rightFrontMotor, Motors.rightRearMotor), wheel_radius_mm=50, heading_offset_tolerance=1)
    bprint("Calibrating Gyro...")
    Sensors.inertial.calibrate()
    while Sensors.inertial.is_calibrating():
        pass
    cclear()
    # Set up controller callbacks here to avoid triggering them by pressing buttons during setup
    # Primary controller bindings
    # Controllers.primary.buttonA.pressed(callback)
    # Secondary controller bindings
    # Controllers.secondary.buttonA.pressed(callback)
    Globals.SETUP_COMPLETE = True
    cprint("Setup complete")
    bprint("Setup complete")
    Controllers.primary.rumble(".")
    Controllers.secondary.rumble(".")
