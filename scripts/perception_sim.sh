#!/bin/bash

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'gate'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 3.0, y: 1.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
isDetected: true
estimateConverged: true
estimateFucked: false" &


sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'buoy'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 7.5, y: -2.6, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
isDetected: true
estimateConverged: true
estimateFucked: false" &

sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'torpedo_poster'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 12.0, y: 4.65, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
isDetected: true
estimateConverged: true
estimateFucked: false" &


sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'torpedo_target'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 12.0, y: 4.65, z: -1.8}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
isDetected: true
estimateConverged: true
estimateFucked: false" &


sleep 10


rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'octagon'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 17.0, y: 2.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
isDetected: true
estimateConverged: true
estimateFucked: false" &
