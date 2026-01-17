# Pursuit Fly Catcher

## Overview

This package implements a **velocity-based pursuit controller** for a drone that attempts to intercept (“catch”) a moving target (the *fly*).

The controller continuously predicts where the fly will be in the near future and steers the drone toward that predicted interception point rather than chasing the fly’s past position.

---

## System Requirements

### Software

* **ROS 2** Humble with the packages: `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `fly_catcher_interfaces` (custom package that is included in the file)   
* Python + `numpy` package
* `numpy`


---

## The Control Approach

to actually catch the fly the most efficient way is it is neccessarry to point the drone

not to the current position of the fly but to calculate where it is going to be this will make 

the drone trajectory shorter and a will allow a more accurate catch that enables overcoming the latency.

to achieve that we will use the kalman filter.

Once the predicted coordinate is known, the distance and angle to that posit will be found and the input control signal will be sent to the drone 

## State Estimation – Kalman Filter

### State Vector

The Kalman filter tracks a **6D constant-velocity state**:

```
x = [px, py, pz, vx, vy, vz]
```


### Motion Model (Constant Velocity)

The fly is modeled with a **constant velocity assumption** between updates:

```
xₖ₊₁ = F xₖ + wₖ
```

Where:

```
F =
[ 1   0   0   dt   0   0]
[ 0   1   0   0   dt   0]
[ 0   0   1   0   0   dt]
[ 0   0   0   1   0   0 ]
[ 0   0   0   0   1   0 ]
[ 0   0   0   0   0   1 ]
```

This integrates velocity into position over time step `dt`.

where F represents my dynamic model,and wₖ represents the process noise

---

### Measurement Model

Each update provides **both position and velocity**:

```
z = [px, py, pz, vx, vy, vz]
```
Note the even thogh the drone is subscribed only to the location of the fly,

the velocity is being determined by differentiated location over time

the timestamps are constanst (10Hz) hence dt = 0.1

Therefore, the observation matrix is:

```
H = I₆
```

This means that Every state variable is observed

---

### Noise Models

* **Process Noise (Q)**
  Models uncertainty in the fly’s motion (unexpected acceleration and maneuvers)

* **Measurement Noise (R)**
  Models sensor noise in position and velocity measurements

Both `Q` and `R` are configured as **ROS parameters**, allowing live tuning without code changes.

Also because it is a kinematic simulator with out a any actual sensor noise or enviroment noise

both of those where set relatively low' but can be adjusted in the YAML file or with ROS CLI command to change parameters.

---

## Control Logic

### 1. Distance Computation
this is for dynamic look ahead calculation, and to determine how to set the the angles and the thrust of the drone.


### 2. Predictive Look-Ahead

The controller predicts where the fly will be after a short horizon,basically meanning- how much ahead do i trust my prediction.

in this code logic the more i get closer to the fly the shorter i will look ahead:

```
look_ahead = clip(real_dist * ratio, min_look_ahead, max_look_ahead)
p_predicted = fly_pos + fly_vel * look_ahead
```

Also is adds alot to the roubustness of the solution and prevents instability and overshoot


---

### 3. Direction Control (Yaw & Pitch)

The drone aims toward the **predicted fly position**:

```
yaw   = atan2(y, x)
pitch = atan2(z, horizontal_distance)
```

This ensures interception rather than chasing the fly’s past location.

---

### 4. Thrust Control (P + Feed-Forward)

```
thrust = (k_ff * |fly_velocity|) + (kp * distance)
```

* **Feed-forward term**: matches fly speed
* **Proportional term**: closes distance error

This control enables fast response to a moving target. Its adding to the stibility and also preventing from following the fly, but not catching it.


---

### 5. Catch Detection & Hold Mode

If the distance is lower then the threshold then:


* Thrust is set to zero
* Catch time and position are logged
* Optional `hold_after_catch` mode keeps the drone powered down

---


## Parameters (Runtime Tunable)

| Parameter          | Description                  |
| ------------------ | ---------------------------- |
| `dt`               | Control loop period          |
| `q_noise`          | Process noise magnitude      |
| `r_noise`          | Measurement noise magnitude  |
| `kp_dist`          | Distance proportional gain   |
| `k_ff_vel`         | Velocity feed-forward gain   |
| `look_ahead_ratio` | Prediction scaling factor    |
| `min_look_ahead`   | Minimum prediction horizon   |
| `max_look_ahead`   | Maximum prediction horizon   |
| `min_thrust`       | Minimum motor thrust         |
| `max_thrust`       | Maximum motor thrust         |
| `catch_threshold`  | Distance threshold for catch |
| `hold_after_catch` | Disable motion after catch   |

---

## Summery of Execution Flow

1. Receive fly position measurement
2. Compute measured velocity
3. Run Kalman prediction and update
4. Compute distance to fly
5. Predict future fly position
6. Compute thrust, yaw, and pitch
7. Publish drone command
8. Publish visualization markers
9. Detect and log catch

---

## How To Run

1. Enter your ros2 workspace:

    ` cd ros2_ws/src/ `

2. clone this repository or extract the zip file the that location
3. go bcack to ros2_ws build and source your workspace with:

    `cd ..`

    `colcon build`

    `source install/setup.bash`

4. Run the launch file:

    `ros2 launch fly_catcher_controller velocity_catching.launch.py`


    this will open the visualization and will strat the program at the same time


## How To tune parameters

### First method - YAML file

Enter the Yaml file in the `config` directory and change them as you wish.

When finish build and source your workspace and then execute the launch command

    ros2 launch fly_catcher_controller velocity_catching.launch.py

## Second method - CLI

all of the node parameters had been added to the launch file, so it is possible to set when launching( also the fly velocity and initial location has been added for QA purposes - feel free to use them as well). 

For example:

    ros2 launch fly_catcher_controller velocity_catching.launch.py   fly_speed:=1.2   kp_dist:=40.0   k_ff_vel:=60.0   look_ahead_ratio:=0.4







