#!/usr/bin/env python3
# Import libraries
import smbus2
import pandas
import numpy


class ThrusterInterfaceAUVDriver:

    def __init__(self,
                 I2C_BUS=1,
                 PICO_I2C_ADDRESS=0x21,
                 PWM_MIN=1100,
                 PWM_MAX=1900,
                 STARTING_PLACE_FOR_PWM=0,
                 SYSTEM_OPERATIONAL_VOLTAGE=16.0,
                 ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET="",
                 THRUSTER_MAPPING=[7, 6, 5, 4, 3, 2, 1, 0],
                 THRUSTER_DIRECTION=[1, 1, 1, 1, 1, 1, 1, 1],
                 THRUSTER_OFFSET=[80, 80, 80, 80, 80, 80, 80, 80]):
        # Initialice the PCA9685 PCB
        self.bus = smbus2.SMBus(I2C_BUS)
        self.PICO_I2C_ADDRESS = PICO_I2C_ADDRESS

        # Set variables for the object to controll PCA9685 PCB
        self.PWM_MIN = PWM_MIN
        self.PWM_MAX = PWM_MAX
        self.STARTING_PLACE_FOR_PWM = STARTING_PLACE_FOR_PWM

        # Set mapping, direction and offset for the thrusters
        self.THRUSTER_MAPPING = THRUSTER_MAPPING
        self.THRUSTER_DIRECTION = THRUSTER_DIRECTION
        self.THRUSTER_OFFSET = THRUSTER_OFFSET

        # Convert SYSTEM_OPERATIONAL_VOLTAGE to a whole even number to work with
        # This is because we have multiple files for the behavious of the thrusters depending on the voltage of the drone
        if (SYSTEM_OPERATIONAL_VOLTAGE < 11.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 10
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 13.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 12
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 15.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 14
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 17.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 16
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 19.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 18
        elif (SYSTEM_OPERATIONAL_VOLTAGE >= 19.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 20

        # Get the full path to the ROS2 package this file is located at
        self.ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET = ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET

    def _interpolate_forces_to_pwm(self, thruster_forces_array):
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Returns an Array of PWM
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """
        # Read the important data from the .csv file
        thrusterDatasheetFileData = pandas.read_csv(
            f"{self.ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET}/resources/T200-Thrusters-{self.SYSTEM_OPERATIONAL_VOLTAGE}V.csv",
            usecols=[' PWM (µs)', ' Force (Kg f)'])

        # Convert Newtons to Kg as Thruster Datasheet is in Kg format
        for i, thruster_forces in enumerate(thruster_forces_array):
            thruster_forces_array[i] = thruster_forces / 9.80665

        # Interpolate data
        thrusterDatasheetFileForces = thrusterDatasheetFileData[
            ' Force (Kg f)'].values
        thrusterDatasheetFileDataPWM = thrusterDatasheetFileData[
            ' PWM (µs)'].values
        interpolated_pwm = numpy.interp(thruster_forces_array,
                                        thrusterDatasheetFileForces,
                                        thrusterDatasheetFileDataPWM)

        # Convert PWM signal to integers as they are interpolated and can have floats
        interpolated_pwm = [int(pwm) for pwm in interpolated_pwm]

        return interpolated_pwm

    def drive_thrusters(self, thruster_forces_array):
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Converts Forces to PWM signals
        PWM signals sent to PCA9685 module through I2C
        PCA9685 Module sends electrical PWM signals to the individual thruster ESCs
        The ESCs send corecponding electrical power to the Thrustres
        Thrusters then generate thrust acordingly to the Forces sent to this driver

        Returns an Array of PWM signal for debugging purposes
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """

        # Apply thruster mapping and direction
        thruster_forces_array = [
            thruster_forces_array[i] * self.THRUSTER_DIRECTION[i]
            for i in self.THRUSTER_MAPPING
        ]

        # Convert Forces to PWM
        thruster_pwm_array = self._interpolate_forces_to_pwm(
            thruster_forces_array)

        # Apply thruster offset
        for i, thruster_pwm in enumerate(thruster_pwm_array):
            thruster_pwm_array[i] = thruster_pwm + self.THRUSTER_OFFSET[i]

        data_to_send = bytes[thruster_pwm_array]

        try:
            self.bus.write_i2c_block_data(self.PICO_I2C_ADDRESS, 0,
                                          list(data_to_send))
        except Exception as e:
            print(f"Failed to send PWM values: {e}")

        return thruster_pwm_array
