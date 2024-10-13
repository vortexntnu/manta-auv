#!/usr/bin/python3
"""
! NOTE:
! For now we don't have a external sensor to measure internal temperature
! Instead we just use Internal Computer temperature sensor to gaugue temperature of the environment approximately
! In the future someone should implement a external temperature sensor for measuting a more accurate state of the temperatuer on the inside of the AUV
"""

# Python Libraries
import subprocess


class TemperatureSensor:
    def __init__(self) -> None:
        # Temperature Sensor Setup
        self.temperature_sensor_file_location = "/sys/class/thermal/thermal_zone0/temp"

    def get_temperature(self) -> float:
        """
        Gets the current temperature from the internal computer's sensor.

        This method reads the temperature value from the internal sensor file, which is in milli°C,
        converts it into Celsius, and returns the result.

        Returns:
        --------
        float
            The current temperature in Celsius. If an error occurs, it returns 0.0.
        """
        try:
            # Read internal temperature on the computer
            result = subprocess.run(
                ["cat", self.temperature_sensor_file_location],
                capture_output=True,
                text=True,
                check=False,
            )

            # Decode and strip to get rid of possible newline characters
            temperature_str = result.stdout.strip()

            # Convert data to float
            temperature = int(temperature_str)

            # We get values in milli°C (thousandths of a Celsius), so we convert it to Celsius
            temperature = temperature / 1000

            return temperature  # In Celsius
        except Exception as error:
            print(f"ERROR: Failed to get temperature: {error}")
            return 0.0  # In Celsius
