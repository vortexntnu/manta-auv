# Libraries for Power Sense Module
import time

import smbus
from MCP342x import MCP342x


class PowerSenseModule:
    def __init__(self) -> None:
        # Parameters
        # to read voltage and current from ADC on PDB through I2C
        self.i2c_adress = 0x69

        # init of I2C bus communication
        self.bus = None
        try:
            self.bus = smbus.SMBus(1)
        except Exception as error:
            print(f"ERROR: Failed to connect to the I2C: {error}")
        time.sleep(1)  # A short pause because sometimes I2C is slow to connect

        # Connect to the PSM through I2C
        self.channel_voltage = None
        self.channel_current = None
        try:
            self.channel_voltage = MCP342x(self.bus, self.i2c_adress, channel=1, resolution=18)  # voltage
            self.channel_current = MCP342x(self.bus, self.i2c_adress, channel=0, resolution=18)  # current
        except Exception as error:
            print(f"ERROR: Failed connecting to PSM: {error}")

        # Conversion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
        self.psm_to_battery_voltage = 11.0  # V/V
        self.psm_to_battery_current_scale_factor = 37.8788  # A/V
        self.psm_to_battery_current_offset = 0.330  # V

    def get_voltage(self) -> float:
        """
        Retrieves the system voltage by reading and converting the channel voltage.

        This method attempts to read the voltage from the power sense module (PSM) and
        convert it to the system voltage using a predefined conversion factor. If an
        I/O timeout or error occurs during the read operation, the method catches the
        exception, logs an error message, and returns a default voltage value of 0.0.

        Returns:
            float: The system voltage if successfully read and converted, otherwise 0.0.
        """
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:
            system_voltage = self.channel_voltage.convert_and_read() * self.psm_to_battery_voltage
            return system_voltage
        except Exception as error:
            print(f"ERROR: Failed retrieving voltage from PSM: {error}")
            return 0.0

    def get_current(self) -> float:
        """
        Retrieves the current system current by reading from the current channel,
        applying an offset, and scaling the result.

        Returns:
            float: The calculated system current. Returns 0.0 if an error occurs.

        Raises:
            Exception: If there is an error in reading or converting the current.
        """
        try:
            system_current = (self.channel_current.convert_and_read() - self.psm_to_battery_current_offset) * self.psm_to_battery_current_scale_factor
            return system_current
        except Exception as error:
            print(f"ERROR: Failed retrieving current from PSM: {error}")
            return 0.0
