#!/bin/bash

echo "##ARMING##";
rostopic pub /thrust/desired_forces geometry_msgs/Wrench "force:
  x: 0.0
  y: 0.0
  z: 0.0
torque:
  x: 0.0
  y: 0.0
  z: 0.0";
echo "ARMING DONE" ;
