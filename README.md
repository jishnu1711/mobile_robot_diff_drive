# mobile_robot_diff_drive

A differential drive robot using **ESP32 + micro-ROS + ros2_control**, driving two **Kinco FD134S** servo drives over CAN (CANopen). Built as a learning platform to understand the micro-ROS/XRCE-DDS stack before exploring agent-less alternatives like Zenoh.

---

## Architecture

```
teleop_twist_keyboard
        │
        │  /diff_drive_controller/cmd_vel  (Twist)
        ▼
DiffDriveController  ──────────────────────────────────────────────────────┐
        │                                                                   │
        │  write(): [left_rad/s, right_rad/s]                              │
        ▼                                                                   │
MicroRosHardware (ros2_control SystemInterface plugin)                     │
        │                                                                   │
        │  /wheel_vel_cmd  (Float64MultiArray)                             │
        ▼                                                                   │
micro-ROS Agent (serial, 576000 baud)                                      │
        │                                                                   │
        │  UART2 (GPIO 16/17)                                              │
        ▼                                                                   │
ESP32 (FreeRTOS + micro-ROS)                                               │
        │                                                                   │
        │  TWAI CAN (GPIO 21/22, 125 kbps)                                │
        ▼                                                                   │
Kinco FD134S ×2  (CANopen, node 0x06=left, 0x07=right)                   │
        │                                                                   │
        │  /esp32/joint_states  (JointState, 50 Hz)                       │
        └───────────────────────────────────────────────────────────────────┘
                                        │
                                        │  read()
                                        ▼
                              DiffDriveController
                                        │
                              /odom  +  odom→base_footprint TF
```

### Key design decisions

- **No kinematics on the ESP32** - `DiffDriveController` computes per-wheel rad/s from the Twist command. The hardware interface publishes those directly as `[left_rad/s, right_rad/s]` on `/wheel_vel_cmd`. The ESP32 only converts rad/s → Kinco DEC units.
- **Separate feedback topic** - ESP32 publishes to `/esp32/joint_states` (not `/joint_states`) to avoid a circular feedback loop with `joint_state_broadcaster`.
- **Name-based joint mapping** - the hardware interface matches ESP32 joint names (`left_wheel`, `right_wheel`) to URDF joint names (`base_left_wheel_joint`, `base_right_wheel_joint`) by substring, not by index.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 (ESP-IDF v5.2) |
| Servo drives | 2× Kinco FD134S |
| CAN bus | TWAI, 125 kbps, GPIO TX=21 RX=22 |
| micro-ROS transport | UART0, 576000 baud |
| Wheel diameter | 150 mm (radius = 0.075 m) |
| Wheel separation | 490 mm |
| Encoder | 2500 PPR incremental, ×4 quadrature = 10,000 counts/rev |

## Repository structure

```
Mobile_Robot_Diff_Drive/
├── README.md
├── .gitignore
├── esp32/                          # ESP-IDF firmware
│   ├── CMakeLists.txt              # root cmake - sets EXTRA_COMPONENT_DIRS
│   ├── sdkconfig                   # ESP-IDF menuconfig (committed intentionally)
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   ├── app_main.c              # entry point
│   │   ├── can_cfg.h               # CAN hardware + node ID config
│   │   ├── robot_config.h          # wheel geometry, scaling constants, topics
│   │   ├── canOPEN.c               # TWAI init, CANopen bringup, CAN RX task
│   │   ├── kinco_dual.h            # public motor API
│   │   ├── microros_node.h
│   │   ├── microros_node.c         # micro-ROS task, pub/sub, safety timeout
│   │   ├── esp32_serial_transport.h
│   │   └── esp32_serial_transport.c # UART transport for micro-ROS
│   └── components/
│       └── micro_ros_espidf/       # micro-ROS component (clone separately - see below)
└── ros2_control_diff_drive/
    └── src/
        ├── my_robot_description/   # URDF, xacro, RViz config
        ├── my_robot_hardware/      # ros2_control SystemInterface plugin
        └── my_robot_bringup/       # controllers yaml, launch file
```

---

## Velocity scaling

### Command path (ESP32 → Kinco)

Kinco DEC formula from manual §8.2:
```
DEC = (RPM × 512 × Encoder_Resolution) / 1875
```

With 2500 PPR encoder:
```
1 rad/s = 40960 / (2π) ≈ 6519.0 DEC
```

`VEL_RAW_PER_RAD_S = 6519.0` in `robot_config.h`.

### Feedback path (Kinco → ESP32)

TPDO1 speed field (`0x606C`) is in **RPM**:
```
rad/s = RPM × (2π / 60)
```

### Position feedback

TPDO1 position field (`0x6064`) is in raw encoder ticks:
```
radians = ticks × (2π / 10000)
```

---

## Building

### ESP32 firmware

**Prerequisites:**
- ESP-IDF v5.2
- ROS 2 Humble (for micro-ROS build)
- micro_ros_espidf component cloned into `esp32/components/`

```bash
cd esp32/components
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git micro_ros_espidf

cd ../..
source /opt/ros/humble/setup.bash
source $IDF_PATH/export.sh

cd esp32
idf.py build flash monitor
```

> **Note:** `libmicroros.a` is not committed - it is built automatically on first `idf.py build`. This takes 10–15 minutes.

**Important:** `colcon.meta` must have `RMW_UXRCE_TRANSPORT=custom`. See `esp32/components/micro_ros_espidf/colcon.meta`.

### ROS2 workspace

**Prerequisites:**
- ROS 2 Humble
- `ros2_control`, `diff_drive_controller`, `joint_state_broadcaster`

```bash
cd ros2_control_diff_drive
colcon build
source install/setup.bash
```

---

## Running

**Terminal 1 - micro-ROS agent:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 576000
```

**Terminal 2 - launch:**
```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

**Terminal 3 - teleoperation:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel
```

Press `z` several times to reduce speed to ~0.2 m/s before driving.

### Verify

```bash
# check controllers are active
ros2 control list_controllers

# check topics
ros2 topic list

# check wheel velocity commands reaching ESP32
ros2 topic echo /wheel_vel_cmd

# check encoder feedback from ESP32
ros2 topic echo /esp32/joint_states
```

---

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/diff_drive_controller/cmd_vel` | `Twist` | host → controller | velocity command from teleop/nav |
| `/wheel_vel_cmd` | `Float64MultiArray` | host → ESP32 | per-wheel rad/s `[left, right]` |
| `/esp32/joint_states` | `JointState` | ESP32 → host | position (rad) + velocity (rad/s) at 50 Hz |
| `/diff_drive_controller/odom` | `Odometry` | host → ROS graph | odometry from diff drive controller |
| `/joint_states` | `JointState` | host → ROS graph | published by joint_state_broadcaster |
| `/tf` | `TFMessage` | host → ROS graph | odom → base_footprint transform |

---

## Configuration

All robot-specific constants are in `esp32/main/robot_config.h` and `ros2/src/my_robot_bringup/config/my_robot_controllers.yaml`. If you change wheel geometry, update both files.

| Parameter | ESP32 (`robot_config.h`) | ROS2 (`my_robot_controllers.yaml`) |
|---|---|---|
| Wheel radius | `WHEEL_RADIUS_M` | `wheel_radius` |
| Wheel separation | - | `wheel_separation` |
| Publish rate | `UROS_PUBLISH_HZ` | `update_rate` |
| cmd_vel timeout | `CMD_VEL_TIMEOUT_MS` | - |

---

## Safety

- If `/wheel_vel_cmd` is not received for `CMD_VEL_TIMEOUT_MS` (500 ms), the ESP32 calls `kinco_stop_all()`
- If the micro-ROS agent connection is lost, the ESP32 stops both motors and re-enters the agent discovery loop
- On deactivation, `MicroRosHardware` publishes `[0.0, 0.0]` to `/wheel_vel_cmd` before teardown

---

## Motivation

This project was built as preparation for the **GSoC 2026 project: Zephyr Zenoh Integration for ros2_control**. The goal was to understand the current micro-ROS/XRCE-DDS stack its architecture, deployment complexity, and limitations before exploring an agent-less alternative using `zenoh-pico` on Zephyr RTOS and `rmw_zenoh` on the host side.
