#!/usr/bin/python3
# Python Libraries
import subprocess


class IPDriver:
    def __init__(self) -> None:
        # Store command as a list of arguments
        self.cmd = ["hostname", "-I"]

    def get_ip(self) -> str:
        """
        Executes a shell command to retrieve the IP address.

        Returns:
            str: The IP address as a string.
        """
        try:
            # Run the command without shell=True
            ip_bytes = subprocess.check_output(self.cmd, stderr=subprocess.STDOUT)
            ip_str = ip_bytes.decode("utf-8").strip()

            # Split by space and get the first IP
            return ip_str.split()[0]

        except subprocess.CalledProcessError as e:
            # Handle the error appropriately (e.g., log it or raise an exception)
            print(f"Failed to retrieve IP address: {e}")
            return ""
