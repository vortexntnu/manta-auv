#!/usr/bin/python3

import os
import time
import traceback
import glob
import logging
import rospy
import smbus
import board
from std_msgs.msg import Float32

from mcp3422 import MCP3422


class BatteryMonitor:
    def __init__(self):
        rospy.init_node("battery_monitor")

        # Parameters
        # to read voltage and current from ADC on PDB through I2C
        self.i2c_adress = 0x69

        # init of I2C bus communication
        self.bus = smbus.SMBus(1)
        self.channel_voltage = MCP3422(channel=0)  # voltage
        self.channel_current = MCP3422(channel=1)  # current
        time.sleep(1)

        # Convertion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
        self.psm_to_battery_voltage = 11.0  # V/V
        self.psm_to_battery_current_scale_factor = 37.8788  # A/V
        self.psm_to_battery_current_offset = 0.330  # V

        # getting params in the ROS-config file (beluga.yaml)
        self.critical_level = rospy.get_param(
            "/battery/thresholds/critical", default=13.5
        )
        self.warning_level = rospy.get_param(
            "/battery/thresholds/warning", default=14.5
        )

        # Polling intervals in seconds delay
        system_interval = rospy.get_param("/battery/system/interval", 1)
        logging_interval = rospy.get_param("/battery/logging/interval", 5)

        # Local variables
        self.system_voltage = 0.0
        self.system_current = 0.0

        self.system_voltage_state = (
            "No receive"  # should only be "No receive", "Error", "Received"
        )
        self.system_current_state = (
            "No receive"  # should only be "No receive", "Error", "Received"
        )

        self.system_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/system", Float32, queue_size=1
        )

        # create current ROS publisher here, if needed

        # set up callbacks
        self.log_timer = rospy.Timer(
            rospy.Duration(secs=logging_interval), self.log_cb
        )  # for logging on ROS terminal

        self.system_timer = rospy.Timer(
            rospy.Duration(secs=system_interval),
            self.system_cb,  # will update and publish measurements to ROS
        )

        rospy.loginfo("BatteryMonitor initialized")

    def system_cb(self, event):
        """Read voltage of system from bootleg ADC."""

        self.read_PSM_voltage()

        self.system_battery_level_pub.publish(self.system_voltage)

        self.read_PSM_current()
        # publish current here if needed

        if self.system_voltage < self.critical_level:
            rospy.logerr(
                f"Critical voltage: {self.system_voltage}V! Shutting down all active nodes!"
            )
            rospy.logerr(f"HAHA just kidding, let's blow up these batteries!")
            # os.system("rosnode kill -a")

    def log_cb(self, event):
        if self.system_voltage_state == "Received":
            self.log_voltage(self.system_voltage, "system")
        if self.system_voltage_state == "Error":
            rospy.logwarn(f"I2C Bus IOerror")
        if self.system_voltage_state == "No receive":
            rospy.loginfo("No voltage recieved from system yet.")

        # if needed for current, same structure

    def log_voltage(self, voltage, title):
        if voltage == 0:
            rospy.loginfo("Voltage is zero. Killswitch is probably off.")

        elif voltage <= self.warning_level:
            rospy.logwarn("%s voltage: %.3fV" % (title, voltage))

        else:
            rospy.loginfo("%s voltage: %.3fV" % (title, voltage))

    def read_PSM_voltage(self):
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:
            self.system_voltage = (
                self.channel_voltage.get_voltage_from_reading()
                * self.psm_to_battery_voltage
            )

            if self.system_voltage_state != "Received":
                self.system_voltage_state = "Received"

        except IOError:
            self.I2C_error_counter_voltage += 1
            self.system_voltage_state = "Error"
            rospy.logwarn(f"I2C Bus IOerror")  # for debug

    def read_PSM_current(self):
        try:
            self.system_current = (
                self.channel_current.get_voltage_from_reading()
                + self.psm_to_battery_current_offset
            ) * self.psm_to_battery_current_scale_factor

            if self.system_current_state != "Received":
                self.system_current_state = "Received"

        except IOError:
            self.I2C_error_counter_current += 1
            self.system_current_state = "Error"
            # rospy.logwarn(f"I2C Bus IOerror. Voltage error counter : {self.I2C_error_counter_current}")

    def shutdown(self):
        self.system_timer.shutdown()
        self.log_timer.shutdown()
        self.bus.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
