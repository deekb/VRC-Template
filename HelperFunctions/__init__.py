"""
Helper functions for our program
"""
import math

from vex import *

from Constants import Color, DriverControlStyle, DrivetrainType

brain = Brain()


def apply_deadzone(value, deadzone):
    """
    Apply a deadzone to the passed value
    :param value: The value to deadzone
    :type value: float
    :param deadzone: The lowest value for the function to consider "alive"
    :type deadzone: float
    :rtype: float
    :returns: The value with a deadzone applied
    """
    if abs(value) < deadzone:
        return 0
    else:
        return (value - math.copysign(deadzone, value)) / (1 - deadzone)  # Preserve a "live" zone of 0.0-1.0


class SlewLimit:
    """
    Limit the acceleration of a value with a slew limiter
    """

    def __init__(self, max_slew_rate_per_second):
        """
        Initialize a new slew limiter with the specified maximum rate
        :param max_slew_rate_per_second: The maximum rate of acceleration in one second, every time the "update" method is called the value is allowed to increase or decrease by a maximum of this value times the delta time
        :type max_slew_rate_per_second: float
        """
        self.max_slew_rate = max_slew_rate_per_second
        self.previous_value = 0
        self.delta_value = 0
        self.current_time = brain.timer.time(MSEC)
        self.previous_time = self.current_time
        self.delta_time = 0

    def update(self, new_value):
        """
        Update the slew function with the most recent speed value
        :param new_value:
        :return:
        """
        self.delta_value = new_value - self.previous_value
        self.current_time = brain.timer.time(MSEC)
        self.delta_time = current_time - self.previous_time
        self.previous_time = self.current_time
        if abs(self.delta_value) > self.max_slew_rate * (self.delta_time / 1000):
            return self.previous_value + self.max_slew_rate * (self.delta_value / abs(self.delta_value))
        return self.previous_value + self.delta_value


def apply_cubic(value: float, linearity: float) -> float:
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


class PIDMotor:
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = PIDMotor(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the speed if it has not yet reached the desired speed
    :param kd: Kd value for the PID: Higher values reduce the speed of response and limit overshoot
    :param t: Time between PID updates
    """

    def __init__(self, motor_object, kp: float = 0.4, kd: float = 0.05, t: float = 0.01):
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
        self.pid_thread = Thread(self.PID_loop)

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
        Used to run the PID in a new thread: updates the values the PID uses and handles applying those updated speed values to the motor
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
            raise NotImplementedError("Unit not implemented, please use \"PERCENT\"")


class Logging:
    """
    A class that can run multiple logs for different events and store their outputs to the SD card
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
        Send a string to the file, using the log format
        :param string:
        :param function_name:
        """
        self.file_object.write(self.log_format.replace("%s", str(string)).replace("%t", str(brain.timer.time(SECONDS))).replace("%m", str(brain.timer.time(MSEC))).replace("%n", str(function_name)))

    def exit(self):
        """
        Close the log object
        """
        self.file_object.close()
