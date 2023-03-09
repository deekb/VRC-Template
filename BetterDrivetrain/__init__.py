"""
A highly customizable drivetrain with built-in dynamic course correction
"""

from vex import *
from HelperFunctions import cubic_normalize, apply_deadzone
from Constants import DrivetrainType, DriverControlStyle


class Drivetrain:
    """
    A better drivetrain than the default Vex one
    """
    def __init__(self, inertial: Inertial, left_side_motors: tuple, right_side_motors: tuple,
                 heading_offset_tolerance: float, wheel_radius_mm: float, turn_aggression: float = 0.25,
                 correction_aggression: float = 0.1, motor_lowest_speed: int = 1,
                 driver_control_linearity: float = 0.45, driver_control_deadzone: float = 0,
                 movement_slowdown_slope: float = 0.2, driver_control_type: int = DriverControlStyle.TANK,
                 drivetrain_type: int = DrivetrainType.TANK) -> None:
        """
        Initialize a new drivetrain with the specified properties
        :param inertial: The inertial sensor to use for the drivetrain
        :type inertial: Inertial
        :param left_side_motors: A tuple of the motors to the left side of the robot
        :type left_side_motors: tuple
        :param right_side_motors: A tuple of the motors to the right side of the robot
        :type right_side_motors: tuple
        :param heading_offset_tolerance: The delta heading that is acceptable or close enough
        :type heading_offset_tolerance: float
        :param turn_aggression: How aggressive to be while turning
        :type turn_aggression: float
        :param correction_aggression: How aggressive to be while correcting movements
        :type correction_aggression: float
        :param wheel_radius_mm: The radius of the wheels
        :type wheel_radius_mm: float
        :param motor_lowest_speed: The speed at which the motors can just barely spin (normally "1" for accuracy but can be set higher if your drivetrain has more friction)
        :type motor_lowest_speed: int
        :param driver_control_linearity: How close to linearly to map the controllers inputs to the motors outputs during the cubic normalization
        :type driver_control_linearity: float
        :param movement_slowdown_slope: The percent speed to decelerate for every mm closer to the target the robot gets, 1 means decelerate 1% for every 1 mm
        :type movement_slowdown_slope: float
        :param driver_control_deadzone: The minimum value from the controller that should be treated as alive (Nonzero), 0.0-1.0, with 0.0 being no deadzone
        :type driver_control_deadzone: float
        :param drivetrain_type: The drivetrains physical type, currently only tank
        :type drivetrain_type: int
        """
        self.inertial = inertial
        self.left_side = MotorGroup(*left_side_motors)
        self.right_side = MotorGroup(*right_side_motors)
        # TODO: can't use chain here, come up with a better solution
        self.all_wheels = MotorGroup(*chain(left_side_motors, right_side_motors))  # This makes an iterator that yields all the motors and makes a new group out of them
        self.heading_offset_tolerance = heading_offset_tolerance
        self.turn_aggression = turn_aggression
        self.correction_aggression = correction_aggression
        self.wheel_radius_mm = wheel_radius_mm
        self.motor_stall_speed = motor_lowest_speed
        self.wheel_circumference_mm = wheel_radius_mm * math.tau
        self.driver_control_type = driver_control_type
        self.driver_control_linearity = driver_control_linearity
        self.driver_control_deadzone = driver_control_deadzone
        self.movement_slowdown_slope = movement_slowdown_slope
        self.drivetrain_type = drivetrain_type
        self.current_heading = 0
        self.current_x = 0
        self.current_y = 0
        # TODO: remove this and allow passing a log object instead
        self.log = Logging(log_format="[%n]:%m:%s\n", mode="wt", log_name="drivetrain")

    def turn_to_heading(self, desired_heading: float) -> None:
        """
        Turn to an absolute heading using the inertial sensor
        :param desired_heading: The heading to turn to
        :type desired_heading: float
        """
        desired_heading %= 360
        get_heading = self.inertial.heading  # Speeds up the process of getting the heading
        current_heading = get_heading(DEGREES) % 360  # Running modulus on the heading to ensure it is between 0 and 360
        # Determine how far off the robot is and which way is a shorter turn
        left_turn_difference = (current_heading - desired_heading)
        right_turn_difference = (desired_heading - current_heading)
        if left_turn_difference < 0:
            left_turn_difference += 360
        if right_turn_difference < 0:
            right_turn_difference += 360
        if left_turn_difference < right_turn_difference:
            delta_heading = left_turn_difference
        else:
            delta_heading = right_turn_difference
        self.all_wheels.set_velocity(0, PERCENT)
        self.all_wheels.spin(FORWARD)
        while abs(delta_heading) > self.heading_offset_tolerance:
            if left_turn_difference < right_turn_difference:
                delta_heading = left_turn_difference
                if self.drivetrain_type == DrivetrainType.TANK:
                    self.left_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
                    self.right_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed) * -1, PERCENT)
                elif self.drivetrain_type == DrivetrainType.X_DRIVE:
                    # TODO: build an X-Drive robot and test this
                    self.left_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
                    self.right_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
                else:
                    raise RuntimeError("Invalid drivetrain type, please use DrivetrainType.TANK or DrivetrainType.X_DRIVE")
            else:
                delta_heading = right_turn_difference
                if self.drivetrain_type == DrivetrainType.TANK:
                    self.left_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed) * -1, PERCENT)
                    self.right_side.set_velocity(delta_heading * self.turn_aggression + self.motor_stall_speed, PERCENT)
                elif self.drivetrain_type == DrivetrainType.X_DRIVE:
                    self.left_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed) * -1, PERCENT)
                    self.right_side.set_velocity((delta_heading * self.turn_aggression + self.motor_stall_speed) * -1, PERCENT)
                else:
                    raise RuntimeError("Invalid drivetrain type, please use DrivetrainType.TANK or DrivetrainType.X_DRIVE")
            current_heading = get_heading(DEGREES) % 360
            left_turn_difference = current_heading - desired_heading
            right_turn_difference = desired_heading - current_heading
            if left_turn_difference < 0:
                left_turn_difference += 360
            if right_turn_difference < 0:
                right_turn_difference += 360
        self.all_wheels.stop()
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
        initial_wheel_degrees_rotated = (
                                                    get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2  # Get the approximate wheel start rotation of the drivetrain by averaging both side's rotations
        wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2
        distance_traveled = abs((
                                            wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
        self.all_wheels.set_velocity(0, PERCENT)
        self.all_wheels.spin(FORWARD)
        while distance_traveled <= distance_mm:
            #  Update the drivetrain state by getting the degrees the motors have spun and the degrees off course
            wheel_degrees_rotated = (get_left_motor_position(DEGREES) + get_right_motor_position(DEGREES)) / 2
            current_heading = get_heading.heading(DEGREES) % 360
            # Calculate the distance we have traveled, so we can calculate our current target speed
            distance_traveled = abs((
                                                wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
            speed = (max(min((
                                         distance_mm - distance_traveled) * self.movement_slowdown_slope + self.motor_stall_speed, abs(target_speed)) + target_speed, target_speed) - target_speed) * initial_speed_sign
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
        self.all_wheels.stop()
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
        distance_traveled = abs((
                                            wheel_degrees_rotated - initial_wheel_degrees_rotated) / 360 * self.wheel_circumference_mm)
        self.log.log("moved towards " + str(desired_heading) + " with heading accuracy of " + str(delta_heading) + " degrees and distance accuracy of " + str(distance_mm - distance_traveled) + "mm", function_name="move_towards_heading")

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
        left_stick, right_stick = {"x": controller.axis4.position, "y": controller.axis3.position}, \
                                  {"x": controller.axis1.position, "y": controller.axis2.position}
        if self.driver_control_type == DriverControlStyle.TANK:
            left_controller_input = apply_deadzone(left_stick["y"]() / 100, self.driver_control_deadzone)
            right_controller_input = apply_deadzone(right_stick["y"]() / 100, self.driver_control_deadzone)
            left_speed = cubic_normalize(left_controller_input, self.driver_control_linearity) * 100
            right_speed = cubic_normalize(right_controller_input, self.driver_control_linearity) * 100
        elif self.driver_control_type == DriverControlStyle.ARCADE:
            raise NotImplemented("Arcade drive not yet implemented, please us tank drive")
        else:
            raise RuntimeWarning("Invalid drivetrain control style")
        if self.drivetrain_type == DrivetrainType.TANK:
            self.left_side.set_velocity(left_speed, PERCENT)
            self.right_side.set_velocity(right_speed, PERCENT)
        # elif self.drivetrain_type == DrivetrainType.X_DRIVE:
        #     self.left_side_motors.set_velocity(left_speed, PERCENT)
        #     self.right_side_motors.set_velocity(right_speed, PERCENT)
        else:
            raise RuntimeError("Invalid drivetrain type, please use DrivetrainType.TANK or DrivetrainType.X_DRIVE")

    def reset(self) -> None:
        """
        Reset the drivetrain
        """
        self.all_wheels.stop()
        self.all_wheels.set_velocity(0, PERCENT)
        self.inertial.set_heading(0, DEGREES)
        self.current_heading = 0
        self.current_x = 0
        self.current_y = 0

    def set_position(self, x, y) -> None:
        """
        Set the position of the robot, useful after calibration to set it to a specific position without modifying heading
        :param x: The robot's new x coordinate
        :type x: float
        :param y: The robot's new y coordinate
        :type y: float
        """
        self.current_x = x
        self.current_y = y

    def set_heading(self, heading) -> None:
        """
        Set the current heading to the passed heading
        :param heading: The new heading
        """
        self.current_heading = heading
