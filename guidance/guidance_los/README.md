# LOS Guidance System
A ROS2 implementation of Line-of-Sight (LOS) guidance for autonomous underwater vehicles.

## Overview
This package implements a 3D Line-of-Sight (LOS) guidance system with adaptive parameter estimation and third-order filtering. It consists of two main components:

1. LOS Guidance Algorithm (`los_guidance_algorithm.py`)
2. ROS2 Action Server (`los_guidance_action_server.py`)

## Components

### 1. LOS Guidance Algorithm
The core guidance algorithm (`los_guidance_algorithm.py`) implements:
- Distance-based velocity control
- Path-following using LOS guidance
- Third-order reference filtering for smooth commands
- Adaptive sideslip compensation

Key classes:
- `State`: Represents vehicle state (position, orientation, velocity)
- `LOSParameters`: Configuration for LOS guidance behavior
- `FilterParameters`: Configuration for command smoothing
- `ThirdOrderLOSGuidance`: Main guidance algorithm implementation

### 2. ROS2 Action Server
The action server (`los_guidance_action_server.py`) provides:
- ROS2 interface for waypoint navigation
- Parameter handling through ROS2 parameter system
- Odometry processing and state estimation
- Real-time guidance command publishing

## Parameters
Parameters are configured through `los_guidance_params.yaml`:

## Guidance Law Equations

The guidance system implements equations from "Handbook of Marine Craft Hydrodynamics and Motion Control" (Fossen, 2011).

### Path-Tangential Reference Frame
The path-tangential coordinate system {p} has its origin at (x_i, y_i, z_i) with x_p-axis pointing towards the next waypoint. The path-following errors are transformed using:

```latex
[x_e^p]    [ cos(π_v)   0   sin(π_v)] [cos(π_h) -sin(π_h) 0] [x^n]
[y_e^p] = [    0       1       0    ] [sin(π_h)  cos(π_h) 0] [y^n]
[z_e^p]    [-sin(π_v)   0   cos(π_v)] [   0        0      1] [z^n]
