"""
Helper functions for our program
"""
import math

from vex import *

from Constants import Color, DrivetrainControlStyle

brain = Brain()


def apply_deadzone(value: float, dead_zone: float = 0.05):
    """
    Apply a deadzone to the passed value
    :param value: The value to deadzone
    :type value: float
    :param dead_zone: The lowest value for the function to consider "alive"
    :type dead_zone: float
    :rtype: float
    :returns: The value with a deadzone applied
    """
    if abs(value) < dead_zone:
        return 0
    return value


class SlewLimit:
    """
    Limit the acceleration of a value with a slew limiter
    """

    def __init__(self, max_slew_rate):
        """
        Initialize a new slew limiter with the specified maximum rate
        :param max_slew_rate: The maximum rate of acceleration in one update, every time the "update" method is called the value is allowed to increase or decrease by a maximum of this value
        :type max_slew_rate: float
        """
        self.max_slew_rate = max_slew_rate
        self.previous_value = 0
        self.delta_value = 0

    def update(self, new_value):
        """
        Update the slew function with the most recent speed value
        :param new_value:
        :return:
        """
        self.delta_value = new_value - self.previous_value
        if self.delta_value and abs(self.delta_value) > self.max_slew_rate:
            return self.previous_value + self.max_slew_rate * (self.delta_value / abs(self.delta_value))
        return self.previous_value + self.delta_value


def cubic_normalize(value: float, linearity: float) -> float:
    """
    Normalize a value across a cubic curve with a linearity
    :param linearity: How close to a linear function the normalizer should use
    :type linearity: float
    :param value: The value to normalize
    :type value: float
    :rtype: float
    :returns: The passed value normalized across a cubic curve
    """
    return value ** 3 + linearity * value / (1 + linearity)


# noinspection PyTypeChecker
# ^ Prevents linter from getting mad about reporting an int but returning an Optional[int] with no "typing" library on the vex brain's python version
def get_optical_color(optical_sensor: Optical) -> int:
    """
    Get the color currently detected by the passed optical sensor
    :param optical_sensor: The sensor to read from
    :type optical_sensor: Optical
    :rtype: int
    :returns: Color.RED, Color.BLUE, Color.YELLOW, or None
    """
    optical_hue = optical_sensor.hue()
    if optical_hue < 30 or optical_hue > 340:
        return Color.RED
    elif 215 < optical_hue < 255:
        return Color.BLUE
    elif 28 < optical_hue < 50:
        return Color.YELLOW
    return


class Drivetrain:
    """
    A better drivetrain than the default Vex one
    """

    def __init__(self, inertial: Inertial, left_side: MotorGroup, right_side: MotorGroup,
                 heading_offset_tolerance: float, wheel_radius_mm: float, turn_aggression: float = 0.5,
                 correction_aggression: float = 0.5, motor_stall_speed: float = 1,
                 driver_control_linearity: float = 0.45, driver_control_deadzone: float = 0.05,
                 movement_slowdown_slope: float = 0.2, driver_control_type: int = DrivetrainControlStyle.TANK) -> None:
        """
        Initialize a new drivetrain with the specified properties
        :param inertial: The inertial sensor to use for the drivetrain
        :type inertial: Inertial
        :param left_side: The motor/motor group corresponding to the left side of the robot
        :type left_side: (Motor | MotorGroup)
        :param right_side: The motor/motor group corresponding to the right side of the robot
        :type right_side: (Motor | MotorGroup)
        :param heading_offset_tolerance: The delta heading that is acceptable or close enough
        :type heading_offset_tolerance: float
        :param turn_aggression: How aggressive to be while turning
        :type turn_aggression: float
        :param correction_aggression: How aggressive to be while correcting movements
        :type correction_aggression: float
        :param wheel_radius_mm: The radius of the wheels
        :type wheel_radius_mm: float
        :param motor_stall_speed: The speed at which the motors can just barely spin (normally "1" for accuracy but can be set higher)
        :type motor_stall_speed: float
        :param driver_control_linearity: How close to linearly to map the controllers inputs to the motors outputs during the cubic normalization
        :type driver_control_linearity: float
        :param movement_slowdown_slope: The percent speed to decelerate for every mm closer to the target the robot gets, 1 means decelerate 1% for every 1 mm
        :type movement_slowdown_slope: float
        :param driver_control_deadzone: The minimum value from the controller that should be treated as alive (Nonzero), 0.0-1.0, with 0.0 being no deadzone
        :type driver_control_deadzone: float
        """
        self.inertial = inertial
        self.left_side = left_side
        self.right_side = right_side
        self.heading_offset_tolerance = heading_offset_tolerance
        self.turn_aggression = turn_aggression
        self.correction_aggression = correction_aggression
        self.wheel_radius_mm = wheel_radius_mm
        self.motor_stall_speed = motor_stall_speed
        self.wheel_circumference_mm = wheel_radius_mm * math.pi * 2
        self.driver_control_type = driver_control_type
        self.driver_control_linearity = driver_control_linearity
        self.driver_control_deadzone = driver_control_deadzone
        self.movement_slowdown_slope = movement_slowdown_slope
        self.current_heading = 0
        self.current_x = 0
        self.current_y = 0
        self.log = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="drivetrain")

    def turn_to_heading(self, desired_heading: float) -> None:
        """
        Turn to a heading (absolute or relative) using the inertial sensor
        :param desired_heading: The heading to turn to
        :type desired_heading: float
        """
        desired_heading %= 360
        get_heading = self.inertial.heading  # Speeds up the process of getting the heading
        current_heading = get_heading(DEGREES) % 360
        left_turn_difference = (current_heading - desired_heading)
        right_turn_difference = (desired_heading - current_heading)
        if left_turn_difference < 0:
            left_turn_difference = left_turn_difference + 360
        if right_turn_difference < 0:
            right_turn_difference = right_turn_difference + 360
        if left_turn_difference < right_turn_difference:
            delta_heading = left_turn_difference
        else:
            delta_heading = right_turn_difference
        self.left_side.set_velocity(0, PERCENT)
        self.right_side.set_velocity(0, PERCENT)
        self.left_side.spin(FORWARD)
        self.right_side.spin(FORWARD)
        while abs(delta_heading) > self.heading_offset_tolerance:
            if left_turn_difference < right_turn_difference:
                delta_heading = left_turn_difference
                self.left_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
                self.right_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed)
                                             * -1, PERCENT)
            else:
                delta_heading = right_turn_difference
                self.left_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed)
                                            * -1, PERCENT)
                self.right_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
            current_heading = get_heading(DEGREES) % 360
            left_turn_difference = current_heading - desired_heading
            right_turn_difference = desired_heading - current_heading
            if left_turn_difference < 0:
                left_turn_difference += 360
            if right_turn_difference < 0:
                right_turn_difference += 360
        self.left_side.stop()
        self.right_side.stop()
        self.current_heading = desired_heading
        wait(500)
        current_heading = get_heading(DEGREES) % 360
        left_turn_difference = (current_heading - desired_heading)
        right_turn_difference = (desired_heading - current_heading)
        if left_turn_difference < 0:  # Ensure that the values are in range -180 to 180
            left_turn_difference += 360
        if right_turn_difference < 0:
            right_turn_difference += 360
        if abs(left_turn_difference) < abs(right_turn_difference):  # Turn towards the most efficient direction
            delta_heading = left_turn_difference
        else:
            delta_heading = right_turn_difference
        self.log.log("turned to " + str(desired_heading) + " with accuracy of " + str(delta_heading) + " degrees", function_name="turn_to_heading")

    def move_towards_heading(self, desired_heading: float, target_speed: float, distance_mm: float) -> None:
        """
        Move towards a heading using dynamic course correction
        :param desired_heading: The absolute heading to move towards
        :type desired_heading: float
        :param target_speed:  The base speed to move at
        :type target_speed: float
        :param distance_mm: The distance to move before stopping the movement
        :type distance_mm: float
        """
        if not (target_speed and distance_mm):  # if the speed or distance is none exit
            raise RuntimeError("Both speed and distance must be nonzero")
        if distance_mm < 0:
            raise RuntimeError("Distance must be positive, to drive in reverse please set speed to a negative")
        get_heading = self.inertial.heading  # Speeds up the process of getting the heading
        get_left_motor_position = self.left_side.position  # Speeds up the process of getting the distance traveled
        get_right_motor_position = self.right_side.position  # Speeds up the process of getting the distance traveled
        desired_heading %= 360
        initial_speed_sign = target_speed / abs(target_speed)  # Equivalent of "sign" in python, only works if target speed is nonzero
        initial_wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2  # Get the approximate wheel start rotation of the drivetrain by averaging both side's rotations
        wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2
        distance_traveled = abs((wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
        self.left_side.set_velocity(0, PERCENT)all the wheels
        self.right_side.set_velocity(0, PERCENT)
        self.left_side.spin(FORWARD)
        self.right_side.spin(FORWARD))
        while distance_traveled <= distance_mm:
            #  Update the drivetrain state by getting the degrees spun and degrees off course
            wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2
            current_heading = get_heading.heading(DEGREES) % 360
            # Calculate the distance we have traveled, so we can calculate our current target speed
            distance_traveled = abs((wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
            speed = (max(min((distance_mm - distance_traveled) * self.movement_slowdown_slope + self.motor_stall_speed, abs(target_speed)) + target_speed, target_speed) - target_speed) * initial_speed_sign
            self.log.log(speed, "speed")
            left_turn_difference = (current_heading - desired_heading)
            right_turn_difference = (desired_heading - current_heading)
            if left_turn_difference < 0:  # Ensure that the values are in range -180 to 180
                left_turn_difference += 360
            if right_turn_difference < 0:
                right_turn_difference += 360
            if abs(left_turn_difference) < abs(right_turn_difference):  # Turn towards the most efficient direction
                delta_heading = left_turn_difference
                self.left_side.set_velocity(delta_heading * self.correction_aggression + speed, PERCENT)
                self.right_side.set_velocity((delta_heading * self.correction_aggression - speed) * -1, PERCENT)
            else:
                delta_heading = right_turn_difference
                self.left_side.set_velocity((delta_heading * self.correction_aggression - speed) * -1, PERCENT)
                self.right_side.set_velocity(delta_heading * self.correction_aggression + speed, PERCENT)
        self.left_side.stop()
        self.right_side.stop()
        self.current_heading = desired_heading
        self.current_x += math.cos(desired_heading * math.pi / 180) * distance_mm
        self.current_y += math.sin(desired_heading * math.pi / 180) * distance_mm
        wait(500)
        current_heading = get_heading.heading(DEGREES) % 360
        left_turn_difference = (current_heading - desired_heading)
        right_turn_difference = (desired_heading - current_heading)
        if left_turn_difference < 0:  # Ensure that the values are in range -180 to 180
            left_turn_difference += 360
        if right_turn_difference < 0:
            right_turn_difference += 360
        if abs(left_turn_difference) < abs(right_turn_difference):  # Turn towards the most efficient direction
            delta_heading = left_turn_difference
        else:
            delta_heading = right_turn_difference
        wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2
        distance_traveled = abs((wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
        self.log.log("moved towards " + str(desired_heading) + " with heading accuracy of " + str(delta_heading) + " degrees and distance accuracy of " + str(distance_mm - distance_traveled) + "mm", function_name="move_towards_heading")

    def turn_relative(self, desired_heading: float) -> None:
        """
        Turn relative to the last operation's target end point using the inertial sensor
        :param desired_heading: How many degrees to turn
        :type desired_heading: float
        """
        self.turn_to_heading(self.current_heading + desired_heading)
    
    def move_to_position(self, x: float, y: float, target_speed: float) -> None:
        """
        Move to an x, y position
        :param x: The x position to mave to
        :param y: The y position to mave to
        :param target_speed: The speed to move at
        """
        angle = math.atan2(x - self.current_x, y - self.current_y) * math.pi / 180
        distance = sqrt(((x - self.current_x) ** 2 + (y - self.current_y) ** 2))
        self.turn_to_heading(desired_heading=angle)
        self.move_towards_heading(desired_heading=angle, target_speed=target_speed, distance_mm=distance)

    def move_with_controller(self, controller: Controller) -> None:
        """
        Move using the controller input
        :param controller: The controller to get input from
        :type controller: Controller
        """
        left_stick, right_stick = {"x": controller.axis4.position, "y": controller.axis3.position}, {
            "x": controller.axis1.position, "y": controller.axis2.position}
        if self.driver_control_type == DrivetrainControlStyle.TANK:
            left_controller_input = apply_deadzone(left_stick["y"]() / 100, self.driver_control_deadzone)
            right_controller_input = apply_deadzone(right_stick["y"]() / 100, self.driver_control_deadzone)
            left_speed = cubic_normalize(left_controller_input, self.driver_control_linearity) * 100
            right_speed = cubic_normalize(right_controller_input, self.driver_control_linearity) * 100
        elif self.driver_control_type == DrivetrainControlStyle.ARCADE:
            raise NotImplemented("Arcade drive not yet implemented, please us tank drive")
        else:
            raise RuntimeWarning("Invalid drivetrain control style")
        self.left_side.set_velocity(left_speed, PERCENT)
        self.right_side.set_velocity(right_speed, PERCENT)

    def reset(self):
        """
        Reset the heading
        """
        self.inertial.set_heading(0, DEGREES)
        self.current_heading = 0
        self.current_x = 0
        self.current_y = 0

    def set_position(self, x, y):
        """
        Set the position of the robot, useful after calibration to set it to a specific position without modifying heading
        :param x: The robot's new x coordinate
        :type x: float
        :param y: The robot's new y coordinate
        :type y: float
        """
        self.current_x = x
        self.current_y = y


class CustomPID:
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = CustomPID(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the speed if it has not yet reached the desired speed
    :param kd: Kd value for the PID: Higher values reduce the speed of response and limit overshoot
    :param t: Time between PID updates
    """

    def __init__(self, motor_object, kp: float = 0.4, kd: float = 0.05, t: float = 0.01) -> None:
        self.motor_object = motor_object
        self.kp = kp
        self.kd = kd
        self.t = t
        self.v = 0
        self.e = 0
        self.d = 0
        self.o = 0
        self.e_pr = 0
        self.target_v = 0

    def PID_update(self) -> None:
        """
        Update the PID state with the most recent motor and target velocities and send the normalized value to the motor
        """
        self.v = abs(self.velocity(PERCENT))
        self.e = self.target_v - self.v
        self.d = (self.e - self.e_pr) / self.t
        self.o = self.kp * self.e + self.kd * self.d
        self.e_pr = self.e
        self.motor_object.set_velocity(self.v + self.o, PERCENT)

    def PID_loop(self) -> None:
        """
        Used to run the PID in a new thread: "Thread(<motor to run PID on>.PID_loop)"; it updates the values the PID uses and
        handles applying those updated speed values to the motor
        """
        while True:
            self.PID_update()
            wait(self.t, SECONDS)

    def set_velocity(self, velocity: float, unit: int = PERCENT) -> None:
        """
        Set the motors target velocity using the PID, make sure you run PID_loop in a new thread or this
        will have no effect
        :param velocity: The new target velocity of the motor
        :type velocity: float
        :param unit: The velocity unit - only PERCENT is currently supported
        :type unit: int
        """
        if unit == PERCENT:
            self.target_v = velocity
        else:
            raise NotImplementedError("Unit not implemented: \"" + str(unit) + "\", please use \"PERCENT\"")

    def set_stopping(self, **kwargs) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.set_stopping(kwargs)

    def stop(self) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.stop()

    def spin(self, direction) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.spin(direction)

    def velocity(self, *args) -> float:
        """
        Passthrough method to the "Motor" class
        """
        return self.motor_object.velocity(*args)


class Logging:
    """
    run multiple logs in parallel
    """

    def __init__(self, log_name, log_format: str, mode: str = "w"):
        """
        Create a new instance of the class
        :param log_name: The name to use for the log, a number preceded by a hyphen "-" will be appended to this name to avoid overwriting old logs
        :type log_name: str
        :param log_format: The format for the log, %s for the passed string, %m for time in milliseconds, %t for time in seconds %n for the passed function name (if supplied, otherwise "None" will be used)
        :type log_format: str
        :param mode: The mode you want to open the file in
        :type mode: str
        """
        self.log_format = log_format
        index_dict = {}
        log_number = 0
        try:
            index_dict = eval(open("/Logs/index.json", "r").read())
            if str(log_name) in index_dict:
                log_number = index_dict[str(log_name)]
                index_dict[str(log_name)] += 1
            else:
                index_dict[str(log_name)] = 0
                log_number = 0
            with open("/Logs/index.json", "wt") as file:
                file.write(str(index_dict))
        except (OSError, AttributeError):
            try:
                index_dict[str(log_name)] = 0
                with open("/Logs/index.json", "wt") as file:
                    file.write(str(index_dict))
            except (OSError, AttributeError):
                raise RuntimeError("No SD card, can't initialize log: " + str(log_name))
        self.file_object = open("/Logs/" + str(log_name) + "-" + str(log_number) + ".log", mode)
        self.log("Starting log at " + "/Logs/" + str(log_name) + "-" + str(log_number) + ".log", function_name="logging.__init__")

    def log(self, string, function_name=None):
        """
        Send a string to the file
        :param string:
        :param function_name:
        """
        self.file_object.write(self.log_format.replace("%s", str(string)).replace("%t", str(brain.timer.time(SECONDS))).replace("%m", str(brain.timer.time(MSEC))).replace("%n", str(function_name)))

    def exit(self):
        """
        Close the log object
        """
        self.file_object.close()
