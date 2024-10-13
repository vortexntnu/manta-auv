# Internal Status

This package contains a library which allows us to get current and voltage values from the Power Sense Module (PSM), and a ros node which publishes this data to the whole ROS environment.

## PSM lib

The ASV as a PSM. It takes voltage and current data going into internal electrical instruments on board. This library is a driver, which allows us to get the voltage (in V) and current (in A) values from PSM through I2C.
The I2C address is 0X69.
* Typical values for voltage: around 21V
* Typical values for current: around 0.6A


## PSM node

This ros node just publishes the current and voltage data we get from the lib to the entire ROS environment.
### Publishes to
* /asv/power_sense_module/current for the current data [A]
* /asv/power_sense_module/voltage for the voltage data [V]
