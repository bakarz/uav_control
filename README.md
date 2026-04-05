# PX4 ROS 2 Offboard Controller

## 📌 Overview
This project implements a ROS 2 node for controlling a PX4-based drone in **Offboard mode**.  
The drone performs an **autonomous delivery mission** using a waypoint-based state machine.

The mission includes:
- Takeoff
- Navigation through multiple waypoints
- Landing at each waypoint (delivery simulation)
- Takeoff again and continue mission
- Final completion

---

##  Features
- ROS 2 + PX4 integration using `px4_msgs`
- Offboard control with position setpoints
- State machine-based mission logic
- Automatic yaw control (fixed or waypoint-facing)
- Landing and dwell (pause) at each waypoint
- Smooth takeoff and landing behavior

---

##  State Machine

The system follows these states:

- **IDLE** → Initial state
- **WARMUP** → Send initial setpoints before offboard activation
- **ARM** → Arm the drone and switch to offboard mode
- **TAKEOFF** → Ascend to target altitude
- **MISSION** → Navigate through waypoints
- **LAND** → Descend for delivery
- **DWELL** → Pause on ground
- **DONE** → Mission complete

---

##  Waypoints

Waypoints are defined in **NED coordinates**:

- X → North
- Y → East
- Z → Down (negative = above ground)


---

##  How It Works

1. The node starts and initializes publishers/subscribers.
2. It sends setpoints during warmup.
3. Switches to **Offboard mode** and arms the drone.
4. Takes off to the defined altitude.
5. Navigates to each waypoint:
   - Publishes position setpoints
   - Checks distance to waypoint
6. At each waypoint:
   - Lands
   - Waits (delivery simulation)
   - Takes off again
7. Ends mission after last waypoint.

---

##  Topics Used

### Publishers
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

### Subscribers
- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_status`

---

## 🛠️ Requirements

- ROS 2 (Humble)
- PX4 Autopilot
- Gazebo (for simulation)
- `px4_msgs` package

---

##  Usage

### 1. Build ROS 2 workspace
```bash
colcon build
source install/setup.bash
```

### 2. Start PX4 SITL
```bash
make px4_sitl gz_x500
```

### 3. Run Micro XRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### 4. Run the node
```bash
ros2 run uav_control offboard_controller
```

---

##  Key Parameters

| Parameter | Description |
|----------|------------|
| TAKEOFF_ALTITUDE | Target altitude (NED) |
| WAYPOINT_TOLERANCE | Distance threshold to consider waypoint reached |
| WAYPOINT_DWELL_S | Pause duration at each waypoint |
| LAND_SPEED_Z | Landing speed |
| CTRL_HZ | Control loop frequency |

---

##  Customization

You can modify:
- Waypoints
- Altitude
- Speed
- Dwell time

---

