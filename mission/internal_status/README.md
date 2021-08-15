# Internal Status

This package contains nodes for retrieving internal statuses like battery levels and temperature.

#### Temperature
High temperatures can be dangerous for any electronic device. Monitoring the temperature is therefore important. Temperature gets published to auv/temperature. Measured in celsius

#### Battery
Running out of power in the middle of a mission is less than ideal, on top of possibly damaging the betteries and other components.

This node publishes the battery level using the auv/battery_level publisher. It also logs the info through `rospy.loginfo`. If battery voltages drop beneath a threshold it will start using `rospy.logwarn` instead of `rospyloginfo`. If it falls even lower it will write the voltage using `rospy.logerror`. Measured in volt

## Inputs and Outputs

#### Publishes to:
* /auv/battery_level/xavier of type Int32 in volt
* /auv/battery_level/system of type Int32 in volt
* /auv/temperature of type Int32 in celsius

## Setup instructions

#### Parameters
In beluga.yaml

* `battery/threshold/warning` dictates at what voltage-level the node changes to `rospy.logwarn`

* `battery/threshold/critical` dictates at what voltage-level the node changes to `rospy.logfatal`

* `battery/logging/path` is the path to the Xavier-specific folder from where the voltage-levels are retrieved see more at [docs.nvidia.com](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0E0AG0HA)

* `battery/logging/powersense_dev` is the path to powersense

* `temperature/logging/zones/` are the names of the zones which we want to monitor

* `*/logging/interval` dictates how often the node updates the voltage/temperature, the time given is how many seconds between each update
