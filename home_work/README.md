# Control Engineer Home Assignment: Fly Catcher (ROS 2 Humble)
## Context

You are given a ROS 2 Humble workspace containing two packages:

fly_catcher_interfaces

Provides message definitions (e.g., fly_catcher_interfaces/msg/DroneCmd).

fly_catcher_simulator

fly.py: publishes fly position at 10 Hz on /fly/position (geometry_msgs/msg/PointStamped).

Fly motion is a closed-loop 3D trajectory, speed limited to 1 m/s.

drone.py: subscribes to /drone/cmd (fly_catcher_interfaces/msg/DroneCmd) and publishes drone state at 10 Hz on /drone/state (nav_msgs/msg/Odometry).

Drone speed range 0..3.0 m/s

Drone acceleration limited to 2g

## Goal

Develop a control algorithm that drives the simulated drone to catch the fly as quickly and reliably as possible, using only the available measurements and command interface.

## Inputs (subscriptions)

Your controller must subscribe to:

/fly/position (geometry_msgs/msg/PointStamped) @ 10 Hz

Fly position in world frame: [x, y, z]

/drone/state (nav_msgs/msg/Odometry) @ 10 Hz

Drone position [x, y, z] and velocity [vx, vy, vz]

Outputs (publications)

## Your controller must publish:

/drone/cmd (fly_catcher_interfaces/msg/DroneCmd) @ 10 Hz (or higher is allowed)

thrust in range [0..100]

yaw (rad)

pitch (rad)

Command semantics (simulator side):

yaw + pitch define the desired 3D direction of motion (world frame)

thrust maps to desired speed magnitude (internally limited by the simulator)

## Catch Definition

The fly is considered “caught” when the distance between drone and fly is below a threshold:

||p_fly - p_drone|| < 0.2 m

## Visualization Requirement

Visualize both fly and drone positions in real time, using one of:

RViz2 markers (visualization_msgs/Marker), or

any ROS-native visualization method you prefer.

Minimum expectation:

fly position visible

drone position visible

(optional) intercept point / pursuit line for debugging

## Controller Requirements

Your solution should include:

Stable closed-loop behavior

Works despite fly motion and constraints (speed/accel limits)

Handles measurement update rate (10 Hz)

Reasonable performance

Should catch within a short time (expected: a few seconds to tens of seconds depending on tuning)

Clean ROS 2 implementation

Proper package structure, launch file, parameters (YAML)

Logging and clear topic names

No hard-coded “magic numbers” without comments/params

Safety / robustness

## Deliverables

Please provide:

A new package fly_catcher_controller containing:

The controller node source code (Python or C++)

A launch file to run the controller

A parameters YAML file

A short README describing:

Your control approach (e.g., pursuit, lead/intercept, filtering)

How to run the full demo (sim + controller + visualization)

How to tune parameters

If fly messages stop arriving, the controller should reduce thrust or hold position instead of continuing blindly.
