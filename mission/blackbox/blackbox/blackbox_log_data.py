#!/usr/bin/env python3

# Python Libraries
import csv
import os
import re
import time
from datetime import datetime, timedelta


class BlackBoxLogData:
    def __init__(self, ros2_package_directory: str = "") -> None:
        # Global variables for .csv file manipulation ----------
        # Get the path for the directory where we will store our data
        self.blackbox_data_directory = ros2_package_directory + "blackbox_data/"

        timestamp = time.strftime("%Y-%m-%d_%H:%M:%S")
        data_file_name = "blackbox_data_" + timestamp + ".csv"
        self.data_file_location = self.blackbox_data_directory + data_file_name

        self.csv_headers = [
            "Time",
            "Power Sense Module Current [A]",
            "Power Sense Module Voltage [V]",
            "Pressure Internal [hPa]",
            "Temperature Ambiant [*C]",
            "Thruster Forces 1 [N]",
            "Thruster Forces 2 [N]",
            "Thruster Forces 3 [N]",
            "Thruster Forces 4 [N]",
            "Thruster Forces 5 [N]",
            "Thruster Forces 6 [N]",
            "Thruster Forces 7 [N]",
            "Thruster Forces 8 [N]",
            "PWM 1",
            "PWM 2",
            "PWM 3",
            "PWM 4",
            "PWM 5",
            "PWM 6",
            "PWM 7",
            "PWM 8",
        ]

        # Manage csv files for blackbox data ----------
        # If there are stale old .csv files => Delete oldest ones
        # If .csv files take up to much space => Delete oldest ones
        self.manage_csv_files()

        # Make new .csv file for logging blackbox data ----------
        with open(self.data_file_location, mode="w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(self.csv_headers)

    # Methods for inside use of the class ----------
    def manage_csv_files(self, max_file_age_in_days: int = 7, max_size_kb: int = 3_000_000) -> None:
        """
        Manages CSV files in the blackbox data directory by deleting old files and ensuring the total size does not exceed a specified limit.

        Args:
            max_file_age_in_days (int, optional): The maximum age of files in days before they are deleted. Defaults to 7 days.
            max_size_kb (int, optional): The maximum total size of all CSV files in kilobytes before the oldest files are deleted.

        Returns:
            None

        Raises:
            ValueError: If there is an error parsing the file timestamp.

        Notes:
            - The method first deletes files older than the specified number of days.
            - If the total size of remaining files exceeds the specified limit, it deletes the oldest files until the size is within the limit.
            - The expected filename format for the CSV files is "blackbox_data_YYYY-MM-DD_HH:MM:SS.csv".
        """
        # adjust the max size before you start deleting old files (1 000 000 kb = 1 000 mb = 1 gb)
        current_time = datetime.now()
        older_than_time = current_time - timedelta(days=max_file_age_in_days)

        # Compile a regular expression pattern for matching the expected filename format
        pattern = re.compile(r"blackbox_data_(\d{4}-\d{2}-\d{2}_\d{2}:\d{2}:\d{2})\.csv")

        # List all .csv files in the blackbox data directory
        csv_files = [f for f in os.listdir(self.blackbox_data_directory) if f.endswith(".csv") and f.startswith("blackbox_data_")]

        for csv_file in csv_files:
            match = pattern.match(csv_file)
            # Skip files that do not match the expected format
            if match is None:
                print(f"Invalid filename format, skipping file: {csv_file}")
                continue

            try:
                file_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H:%M:%S")
            except ValueError as e:
                print(f"Error parsing file timestamp, skipping file: {csv_file}. Error: {e}")
                continue

            if file_time < older_than_time:
                file_path = os.path.join(self.blackbox_data_directory, csv_file)
                os.remove(file_path)
                print(f"Deleted old csv file: {file_path}")

        # Calculate the total size of remaining .csv files
        total_size_kb = (
            sum(
                os.path.getsize(os.path.join(self.blackbox_data_directory, f)) for f in os.listdir(self.blackbox_data_directory) if f.endswith(".csv")
            )
            / 1024
        )

        csv_files = [
            f for f in os.listdir(self.blackbox_data_directory) if f.endswith(".csv") and f.startswith("blackbox_data_") and pattern.match(f)
        ]
        # Delete oldest files if total size exceeds max_size_kb
        while total_size_kb > max_size_kb:
            # Sort .csv files by timestamp (oldest first)
            csv_files_sorted = sorted(
                csv_files,
                key=lambda x: datetime.strptime(pattern.match(x).group(1), "%Y-%m-%d_%H:%M:%S"),
            )

            if not csv_files_sorted:
                print("No .csv files to delete.")
                break

            oldest_file = csv_files_sorted[0]
            oldest_file_path = os.path.join(self.blackbox_data_directory, oldest_file)
            os.remove(oldest_file_path)
            print(f"Deleted the oldest csv file: {oldest_file_path}")

            # Recalculate the total size of remaining .csv files
            total_size_kb = (
                sum(
                    os.path.getsize(os.path.join(self.blackbox_data_directory, f))
                    for f in os.listdir(self.blackbox_data_directory)
                    if f.endswith(".csv")
                )
                / 1024
            )
            csv_files.remove(oldest_file)  # Ensure the deleted file is removed from the list
            print(f"Now the total size of .csv files is: {total_size_kb:.2f} KB")

    # Methods for external uses ----------
    def log_data_to_csv_file(
        self,
        psm_current: float = 0.0,
        psm_voltage: float = 0.0,
        pressure_internal: float = 0.0,
        temperature_ambient: float = 0.0,
        thruster_forces_1: float = 0.0,
        thruster_forces_2: float = 0.0,
        thruster_forces_3: float = 0.0,
        thruster_forces_4: float = 0.0,
        thruster_forces_5: float = 0.0,
        thruster_forces_6: float = 0.0,
        thruster_forces_7: float = 0.0,
        thruster_forces_8: float = 0.0,
        pwm_1: int = 0,
        pwm_2: int = 0,
        pwm_3: int = 0,
        pwm_4: int = 0,
        pwm_5: int = 0,
        pwm_6: int = 0,
        pwm_7: int = 0,
        pwm_8: int = 0,
    ) -> None:
        """
        Logs the provided data to a CSV file.
        Parameters:
        - psm_current (float): The current of the power supply module.
        - psm_voltage (float): The voltage of the power supply module.
        - pressure_internal (float): The internal pressure.
        - temperature_ambient (float): The ambient temperature.
        - thruster_forces_1 (float): The force of thruster 1.
        - thruster_forces_2 (float): The force of thruster 2.
        - thruster_forces_3 (float): The force of thruster 3.
        - thruster_forces_4 (float): The force of thruster 4.
        - thruster_forces_5 (float): The force of thruster 5.
        - thruster_forces_6 (float): The force of thruster 6.
        - thruster_forces_7 (float): The force of thruster 7.
        - thruster_forces_8 (float): The force of thruster 8.
        - pwm_1 (int): The PWM signal for thruster 1.
        - pwm_2 (int): The PWM signal for thruster 2.
        - pwm_3 (int): The PWM signal for thruster 3.
        - pwm_4 (int): The PWM signal for thruster 4.
        - pwm_5 (int): The PWM signal for thruster 5.
        - pwm_6 (int): The PWM signal for thruster 6.
        - pwm_7 (int): The PWM signal for thruster 7.
        - pwm_8 (int): The PWM signal for thruster 8.
        This method appends a new row to the CSV file specified by `self.data_file_location`.
        The row contains the current time and the provided data values.
        """
        # Get current time in hours, minutes, seconds and milliseconds
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Write to .csv file
        with open(self.data_file_location, mode="a", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    current_time,
                    psm_current,
                    psm_voltage,
                    pressure_internal,
                    temperature_ambient,
                    thruster_forces_1,
                    thruster_forces_2,
                    thruster_forces_3,
                    thruster_forces_4,
                    thruster_forces_5,
                    thruster_forces_6,
                    thruster_forces_7,
                    thruster_forces_8,
                    pwm_1,
                    pwm_2,
                    pwm_3,
                    pwm_4,
                    pwm_5,
                    pwm_6,
                    pwm_7,
                    pwm_8,
                ]
            )
