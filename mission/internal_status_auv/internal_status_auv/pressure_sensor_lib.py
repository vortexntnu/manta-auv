#!/usr/bin/python3
# Python Libraries
import time

import adafruit_mprls  # ! NOTE: sudo pip3 install adafruit-circuitpython-mprls

# Pressure sensor Libraries
import board


class PressureSensor:
    def __init__(self) -> None:
        # Pressure Sensor Setup
        i2c_adress_mprls = 0x18  # Reads pressure from MPRLS Adafruit sensor
        self.channel_pressure = None

        try:
            i2c_bus = board.I2C()
            self.channel_pressure = adafruit_mprls.MPRLS(
                i2c_bus=i2c_bus,
                addr=i2c_adress_mprls,
                reset_pin=None,
                eoc_pin=None,
                psi_min=0,
                psi_max=25,
            )
        except Exception as error:
            print(f"ERROR: Couldn't connect to Pressure Sensor: {error}")

        time.sleep(1)

    def get_pressure(self) -> float:
        """
        Retrieves the current pressure reading from the sensor.

        This method attempts to read the pressure from the connected MPRLS sensor.
        If the reading is successful, the pressure value is returned.
        If an error occurs, an error message is printed and 0.0 is returned.

        Returns:
        --------
        float
            The pressure reading in hPa (hectopascals), or 0.0 if an error occurs.
        """
        try:
            pressure = self.channel_pressure.pressure
            return pressure
        except Exception as error:
            print(f"ERROR: Failed to get pressure: {error}")
            return 0.0
