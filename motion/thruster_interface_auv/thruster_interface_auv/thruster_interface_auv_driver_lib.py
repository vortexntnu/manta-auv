#!/usr/bin/env python3
# Import libraries
import numpy
import pandas
import smbus2


class ThrusterInterfaceAUVDriver:
    def __init__(
        self,
        i2c_bus: int = 1,
        pico_i2c_address: int = 0x21,
        system_operational_voltage: float = 16.0,
        ros2_package_name_for_thruster_datasheet: str = "",
        thruster_mapping: list[int] = [7, 6, 5, 4, 3, 2, 1, 0],
        thruster_direction: list[int] = [1, 1, 1, 1, 1, 1, 1, 1],
        thruster_pwm_offset: list[int] = [0, 0, 0, 0, 0, 0, 0, 0],
        pwm_min: list[int] = [1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100],
        pwm_max: list[int] = [1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900],
    ) -> None:
        # Initialice the I2C communication
        self.bus = None
        try:
            self.bus = smbus2.SMBus(i2c_bus)
        except Exception as error_code:
            print(f"ERROR: Failed connection I2C bus nr {self.bus}: {error_code}")
        self.pico_i2c_address = pico_i2c_address

        # Set mapping, direction and offset for the thrusters
        self.thruster_mapping = thruster_mapping
        self.thruster_direction = thruster_direction
        self.thruster_pwm_offset = thruster_pwm_offset
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max

        # Convert SYSTEM_OPERATIONAL_VOLTAGE to a whole even number to work with
        # This is because we have multiple files for the behaviours of the thrusters depending on the voltage of the drone
        if system_operational_voltage < 11.0:
            self.system_operational_voltage = 10
        elif system_operational_voltage < 13.0:
            self.system_operational_voltage = 12
        elif system_operational_voltage < 15.0:
            self.system_operational_voltage = 14
        elif system_operational_voltage < 17.0:
            self.system_operational_voltage = 16
        elif system_operational_voltage < 19.0:
            self.system_operational_voltage = 18
        elif system_operational_voltage >= 19.0:
            self.system_operational_voltage = 20

        # Get the full path to the ROS2 package this file is located at
        self.ros2_package_name_for_thruster_datasheet = ros2_package_name_for_thruster_datasheet

    def _interpolate_forces_to_pwm(self, thruster_forces_array: list) -> list:
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Returns an Array of PWM
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """
        # Read the important data from the .csv file
        thruster_datasheet_file_data = pandas.read_csv(
            f"{self.ros2_package_name_for_thruster_datasheet}/resources/T200-Thrusters-{self.system_operational_voltage}V.csv",
            usecols=[" PWM (µs)", " Force (Kg f)"],
        )

        # Convert Newtons to Kg as Thruster Datasheet is in Kg format
        for i, thruster_forces in enumerate(thruster_forces_array):
            thruster_forces_array[i] = thruster_forces / 9.80665

        # Interpolate data
        thruster_datasheet_file_forces = thruster_datasheet_file_data[" Force (Kg f)"].values
        thruster_datasheet_file_data_pwm = thruster_datasheet_file_data[" PWM (µs)"].values
        interpolated_pwm = numpy.interp(
            thruster_forces_array,
            thruster_datasheet_file_forces,
            thruster_datasheet_file_data_pwm,
        )

        # Convert PWM signal to integers as they are interpolated and can have floats
        interpolated_pwm = [int(pwm) for pwm in interpolated_pwm]

        return interpolated_pwm

    def _send_data_to_escs(self, thruster_pwm_array: list) -> None:
        i2c_data_array = []

        # Divide data into bytes as I2C only sends bytes
        # We have 8 values of 16 bits
        # Convert them to 16 values of 8 bits (ie 1 byte)
        for i in enumerate(thruster_pwm_array):
            msb = (thruster_pwm_array[i] >> 8) & 0xFF
            lsb = (thruster_pwm_array[i]) & 0xFF
            i2c_data_array.extend([msb, lsb])

        # Send the whole array through I2C
        # OBS!: Python adds an extra byte at the start that the Microcotroller that is receiving this has to handle
        self.bus.write_i2c_block_data(self.pico_i2c_address, 0, i2c_data_array)

    def drive_thrusters(self, thruster_forces_array: list) -> list:
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Converts Forces to PWM signals
        PWM signals sent to PCA9685 module through I2C
        PCA9685 Module sends electrical PWM signals to the individual thruster ESCs
        The ESCs send corecponding electrical power to the Thrustres
        Thrusters then generate thrust accordingly to the Forces sent to this driver

        Returns an Array of PWM signal for debugging purposes
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """

        # Apply thruster mapping and direction
        thruster_forces_array = [thruster_forces_array[i] * self.thruster_direction[i] for i in self.thruster_mapping]

        # Convert Forces to PWM
        thruster_pwm_array = self._interpolate_forces_to_pwm(thruster_forces_array)

        # Apply thruster offset
        for esc_channel, thruster_pwm in enumerate(thruster_pwm_array):
            thruster_pwm_array[esc_channel] = thruster_pwm + self.thruster_pwm_offset[esc_channel]

        # Apply thruster offset and limit PWM if needed
        for esc_channel in enumerate(thruster_pwm_array):
            # Clamping pwm signal in case it is out of range
            if thruster_pwm_array[esc_channel] < self.pwm_min[esc_channel]:  # To small
                thruster_pwm_array[esc_channel] = self.pwm_min[esc_channel]
            elif thruster_pwm_array[esc_channel] > self.pwm_max[esc_channel]:  # To big
                thruster_pwm_array[esc_channel] = self.pwm_max[esc_channel]

        # Send data through I2C to the microcontroller that then controls the ESC and extension the thrusters
        try:
            self._send_data_to_escs(thruster_pwm_array)
        except Exception as error_code:
            print(f"ERROR: Failed to send PWM values: {error_code}")

        return thruster_pwm_array
