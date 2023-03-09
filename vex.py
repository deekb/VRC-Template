from __future__ import print_function
import builtins as __builtin__
import os
import sys
import time
from threading import Thread

CompetitionState = input('Please enter the competition state. Please enter either "driver_control" or "autonomous":')
SECONDS = "SECONDS"
PERCENT = "PERCENT"
MSEC = "MSEC"
PRIMARY = "PRIMARY"
PARTNER = "PARTNER"
COAST = "COAST"
BRAKE = "BRAKE"
HOLD = "HOLD"



class Competition:
    def __init__(self, driver_control, autonomous):
        if CompetitionState == "driver_control":
            driver_control()
        elif CompetitionState == "autonomous":
            autonomous()
        else:
            print("Unknown Competition State")
            sys.exit(1)


class competition:
    @staticmethod
    def is_driver_control():
        return CompetitionState == "driver_control"

    @staticmethod
    def is_autonomous():
        return CompetitionState == "autonomous"

    @staticmethod
    def is_enabled():
        return True


class Inertial:
    def __init__(self, Port):
        pass


class Controller:
    def __init__(self, port):
        pass


class Motor:
    def __init__(self, Port, GearRatio, Inverted):
        pass

    def spin(self, direction):
        pass


class MotorGroup:
    def __init__(self, *motors):
        pass
    def spin(self, direction):
        pass


class Ports:
    PORT1 = "Port 1"
    PORT2 = "Port 2"
    PORT3 = "Port 3"
    PORT4 = "Port 4"
    PORT5 = "Port 5"
    PORT6 = "Port 6"
    PORT7 = "Port 7"
    PORT8 = "Port 8"
    PORT9 = "Port 9"
    PORT10 = "Port 10"
    PORT11 = "Port 11"
    PORT12 = "Port 12"
    PORT13 = "Port 13"
    PORT14 = "Port 14"
    PORT15 = "Port 15"
    PORT16 = "Port 16"
    PORT17 = "Port 17"
    PORT18 = "Port 18"
    PORT19 = "Port 19"
    PORT20 = "Port 20"
    PORT21 = "Port 21"


class GearSetting:
    RATIO_18_1 = "18 to 1"


class FontType:
    @staticmethod
    def MONO12():
        pass

    @staticmethod
    def MONO15():
        pass

    @staticmethod
    def MONO20():
        pass

    @staticmethod
    def MONO30():
        pass

    @staticmethod
    def MONO40():
        pass

    @staticmethod
    def MONO60():
        pass

    @staticmethod
    def PROP20():
        pass

    @staticmethod
    def PROP30():
        pass

    @staticmethod
    def PROP40():
        pass

    @staticmethod
    def PROP60():
        pass


class Color:
    @staticmethod
    def BLACK():
        pass

    @staticmethod
    def WHITE():
        pass

    @staticmethod
    def RED():
        pass

    @staticmethod
    def GREEN():
        pass

    @staticmethod
    def BLUE():
        pass

    @staticmethod
    def YELLOW():
        pass

    @staticmethod
    def ORANGE():
        pass

    @staticmethod
    def PURPLE():
        pass

    @staticmethod
    def CYAN():
        pass

    @staticmethod
    def TRANSPARENT():
        pass


class Brain:
    class screen:
        @staticmethod
        def print(text):
            print(text)

        @staticmethod
        def set_cursor(ROW, COLUMN):
            pass

        @staticmethod
        def next_row():
            print("\n")

        @staticmethod
        def clear_screen():
            command = 'clear'
            if os.name in ('nt', 'dos'):
                command = 'cls'
            os.system(command)

        @staticmethod
        def clear_row(ROW=-1):
            pass

        @staticmethod
        def draw_pixel(X, Y):
            pass

        @staticmethod
        def draw_line(START_X, START_Y, END_X, END_Y):
            pass

        @staticmethod
        def draw_rectangle(X, Y, WIDTH, HEIGHT):
            pass

        @staticmethod
        def draw_circle(X, Y, RADIUS):
            pass

        @staticmethod
        def set_font(FONT_TYPE):
            pass

        @staticmethod
        def set_pen_width(PEN_WIDTH):
            pass

        @staticmethod
        def set_pen_color(COLOR):
            pass

        @staticmethod
        def set_fill_color(COLOR):
            pass

        @staticmethod
        def pressed(callback):
            callback()

        @staticmethod
        def released(callback):
            callback()

        @staticmethod
        def row():
            return 20

        @staticmethod
        def column():
            return 80

        @staticmethod
        def pressing(callback):
            return True

        @staticmethod
        def x_position():
            return 0

        @staticmethod
        def y_position():
            return 0

    class battery:
        @staticmethod
        def voltage():
            return 12.0

        @staticmethod
        def current():
            return 16.0

        @staticmethod
        def capacity():
            return 100.0

    class timer:
        @staticmethod
        def event(callback, timee):
            time.sleep(timee * 1000)
            callback()

        @staticmethod
        def clear():
            pass

        @staticmethod
        def time(UNITS):
            pass

    class Event:
        @staticmethod
        def broadcast():
            pass

        @staticmethod
        def broadcast_and_wait():
            pass

        @staticmethod
        def __call__(callback):
            callback()

        @staticmethod
        def wait(_time):
            time.sleep(_time)


def print(text):
    if text == r"\033[2J":
        command = 'clear'
        if os.name in ('nt', 'dos'):
            command = 'cls'
        os.system(command)
    __builtin__.print(text)

def wait(_time, _type = MSEC):
    if _type == SECONDS:

    elif _type == MSEC
    time.sleep(milliseconds / 1000)


Brain.screen.print("Hello World")
Brain.screen.clear_screen()
