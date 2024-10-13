#!/usr/bin/python3

# Python Libraries
from time import sleep

from ip_lib import IPDriver

# Custom Libraries
from lcd_lib import LCDScreenDriver
from power_sense_module_lib import PowerSenseModule
from pressure_sensor_lib import PressureSensor
from temperature_sensor_lib import TemperatureSensor

# Initialize all necessary drivers/libraries for the display and functionality
LCD = LCDScreenDriver()
IP = IPDriver()
PSM = PowerSenseModule()
Pressure = PressureSensor()
Temperature = TemperatureSensor()


# Formatting function for nices LCD screen layout
def format_line(value: str, unit: str) -> str:
    """
    Formats a string to fit within a 16-character display, appending a unit with spacing.

    Args:
        value (str): The value to be displayed.
        unit (str): The unit to be appended to the value.

    Returns:
        str: A formatted string that fits within a 16-character limit, with the unit appended.
    """
    spaces_available = 16
    value_length = len(value)
    unit_length = len(unit) + 1  # +1 to make sure there is spacing between value and unit

    empty_space_length = spaces_available - (value_length + unit_length)
    empty_space_length = max(empty_space_length, 0)

    formated_string = value[0 : (spaces_available - unit_length)]
    formated_string += " " * empty_space_length
    formated_string += " " + unit

    return formated_string


# Fancy animation at the start
LCD.fancy_animation(animation_speed=3.0)

# Display information on the LDC Screen
while True:
    # IP ----------
    TIME_DISPLAYING = 5
    UPDATES_PER_SECOND = 1
    for i in range(TIME_DISPLAYING * UPDATES_PER_SECOND):
        LINE_1 = "IP: "
        LINE_2 = str(IP.get_ip())
        LCD.write_to_screen(LINE_1, LINE_2)
        sleep(1 / UPDATES_PER_SECOND)

    # Voltage and Current ----------
    TIME_DISPLAYING = 5
    UPDATES_PER_SECOND = 2
    for i in range(TIME_DISPLAYING * UPDATES_PER_SECOND):
        LINE_1 = format_line(str(round(PSM.get_voltage(), 3)), "V")
        LINE_2 = format_line(str(round(PSM.get_current(), 3)), "A")
        LCD.write_to_screen(LINE_1, LINE_2)
        sleep(1 / UPDATES_PER_SECOND)

    # Pressure and Temperature ----------
    TIME_DISPLAYING = 5
    UPDATES_PER_SECOND = 1
    for i in range(TIME_DISPLAYING * UPDATES_PER_SECOND):
        LINE_1 = format_line(str(round(Pressure.get_pressure(), 1)), "hPa")
        LINE_2 = format_line(str(round(Temperature.get_temperature(), 1)), "*C")
        LCD.write_to_screen(LINE_1, LINE_2)
        sleep(1 / UPDATES_PER_SECOND)
