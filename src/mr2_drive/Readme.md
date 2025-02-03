# Rover Control Nodes

This package contains two ROS2 nodes that together form the control system for a rover. The **SteeringNode** computes the wheel steering angles and speeds based on a target twist, while the **DrivingNode** processes joystick inputs (and potentially autonomous commands) to generate target twist commands.

---

## Table of Contents

- [Overview](#overview)
  - [Steering Node](#steering-node)
  - [Driving Node](#driving-node)
- [Topics and Services](#topics-and-services)
- [Parameters](#parameters)
- [Building and Running](#building-and-running)
- [Usage Examples](#usage-examples)
- [Notes](#notes)

---

## Overview

### Steering Node

The **SteeringNode** is responsible for computing the steering angles and wheel speeds from a given target twist. It implements a kinematic model based on the rover's dimensions (width and length) and publishes the calculated joint states. Additionally, the node supports a debug mode that allows manual input of joint states via a separate topic.

**Key Features:**
- **Target Twist Processing:**  
  Subscribes to `/driving_node/current_twist` to receive target twist messages, calculates the steering angles and wheel speeds, and publishes these on the `/steering_node/joint_states` topic.
  
- **Debug Mode:**  
  When debug mode is enabled via the `/steering_node/set_debug_mode` service, the node will ignore target twist messages and instead use joint state values from the `/steering_node/debug_input` topic.
  
- **Parameters:**  
  - `rover_width` (default: 0.65 m)
  - `rover_length` (default: 0.954 m)

---

### Driving Node

The **DrivingNode** processes joystick input and generates twist commands to control the rover. It supports both manual and autonomous modes and includes a control loop to smoothly transition the rover's velocities based on acceleration limits.

**Key Features:**
- **Joystick Input Processing:**  
  Subscribes to the `/gamepad` topic (`sensor_msgs/msg/Joy`) to receive joystick data. In manual mode, it maps joystick axes to throttle, brake, steering, and velocity offsets.
  
- **Mode Switching:**  
  Provides a service `/driving_node/switch_mode` to switch between manual and autonomous modes. The current mode is published on `/driving_node/is_autonomous` (`std_msgs/msg/Bool`).

- **Velocity Control:**  
  Generates a target twist published on `/driving_node/target_twist` and maintains a controlled current twist (published on `/driving_node/current_twist`) by respecting defined acceleration constraints.

- **Parameters:**  
  Includes several parameters to configure axis mapping, timeouts, maximum velocities, acceleration limits, and control loop intervals. For example:
  - `throttle_index`, `brake_index`, `steering_index`
  - `vx_index`, `vy_index`
  - `throttle_timeout` (ms)
  - `max_vx`, `max_vy`, `max_omega`
  - `max_x_acceleration`, `max_y_acceleration`, `max_lateral_acceleration`
  - `control_interval` (ms)

---

## Topics and Services

### Steering Node

- **Subscriptions:**
  - `/driving_node/current_twist` (`geometry_msgs/msg/Twist`): Target twist for computing joint states.
  - `/steering_node/debug_input` (`sensor_msgs/msg/JointState`): Joint state input used in debug mode.

- **Publications:**
  - `/steering_node/joint_states` (`sensor_msgs/msg/JointState`): Calculated wheel steering angles and speeds.

- **Services:**
  - `/steering_node/set_debug_mode` (`std_srvs/srv/SetBool`): Toggle debug mode. When enabled, the node will use `/steering_node/debug_input` instead of computed values.

### Driving Node

- **Subscriptions:**
  - `/gamepad` (`sensor_msgs/msg/Joy`): Joystick inputs.

- **Publications:**
  - `/driving_node/is_autonomous` (`std_msgs/msg/Bool`): Current mode indicator (true = autonomous, false = manual).
  - `/driving_node/target_twist` (`geometry_msgs/msg/Twist`): Twist command derived from joystick inputs.
  - `/driving_node/current_twist` (`geometry_msgs/msg/Twist`): Smoothed twist command after applying acceleration limits.

- **Services:**
  - `/driving_node/switch_mode` (`std_srvs/srv/SetBool`): Switch between manual and autonomous modes.

---

## Parameters

Both nodes support runtime configuration via ROS2 parameters.

- **Steering Node:**
  - `rover_width` (double): Width of the rover.
  - `rover_length` (double): Length of the rover.

- **Driving Node:**
  - Joystick axis indices: `throttle_index`, `brake_index`, `steering_index`, `vx_index`, `vy_index`.
  - `throttle_timeout` (int): Timeout in milliseconds to determine joystick inactivity.
  - Maximum velocity limits: `max_vx`, `max_vy`, `max_omega`.
  - Acceleration limits: `max_x_acceleration`, `max_y_acceleration`, `max_lateral_acceleration`.
  - `control_interval` (int): Interval in milliseconds for the control loop.

Parameters can be set via command line or parameter files when launching the nodes.
