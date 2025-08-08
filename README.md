# ROS2 Robot Project: Setup & Roadmap

Welcome! This document outlines the recommended architecture, hardware, and phased development plan for your ROS2-based robot project.

---

## 🚦 Architecture Suggestions

- **Control Separation:**  
    Use `ros2_control` + `diff_drive_controller` on the Raspberry Pi. Keep low-level PID/encoders on the Arduino.  
    *Pi = commands/state, Arduino = fast motor loop.*

- **Motor Driver:**  
    Replace L298N (lossy, hot) with TB6612FNG or DRV8833 for better efficiency and current handling.

- **Device Naming:**  
    Add udev rules for persistent device names (`/dev/arduino`, `/dev/lidar`).

- **Safety:**  
    Add a watchdog to stop motors if no `/cmd_vel` within N ms.

- **ROS2 Best Practices:**  
    Use lifecycle nodes, YAML parameters, and add diagnostics (`diagnostic_updater`).

- **Navigation:**  
    Start with Nav2 + `slam_toolbox`; add custom planning later if needed.

- **TF Tree:**  
    Publish a correct TF tree:  
    - `map → odom` (from SLAM)  
    - `odom → base_link` (from wheel odom)  
    - `base_link → laser`

- **Calibration:**  
    Calibrate wheel radius, track width, encoder ticks; verify with closed-loop tests.

- **Testing:**  
    Record rosbag during tests; create RViz config.

- **MCU Upgrade:**  
    Consider Teensy/RP2040 if encoder rates or PID load exceed Uno limits.

---

## 📝 Phased TODO

### **Phase 0 — System Setup**

- Install ROS 2 (Humble/Jazzy) on Pi 5.
- Add user to `dialout` group; enable serial:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
- Create udev rules and reload:
    ```udev
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino", MODE="0666"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
    # ...adjust VID:PID for your boards...
    ```
    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

---

### **Phase 1 — `sensors_pkg`**

- **LD20 Node:**
    - Parse serial frames robustly (sync on start byte, length, checksum).
    - Publish `sensor_msgs/LaserScan` on `/scan` with correct `frame_id` (`laser_frame`) and timestamps.
    - Expose params: port, baudrate, frame_id, range limits, angle min/max.
    - Add diagnostics (rate, packet errors, checksum fails).
- **URDF:**  
    Add `base_link` and laser frame offset (use `robot_state_publisher`).

---

### **Phase 2 — `control_pkg`**

- **Topic Interface:**
    - Subscribe to `geometry_msgs/Twist` on `/cmd_vel`.
    - Convert to left/right wheel target velocities (diff kinematics).
- **Arduino Protocol:**
    - Define compact binary protocol (start byte, payload, checksum).
    - Implement Arduino FW: read target wheel velocities, run PID at 100–200 Hz, read encoders via interrupts, stream wheel velocity/position back.
- **Odometry on Pi:**
    - Fuse wheel ticks into `nav_msgs/Odometry`; publish TF `odom→base_link` at 30–50 Hz.
    - Params: wheel_radius, track_width, ticks_per_rev, gear ratio, encoder polarity.
- **Safety:**
    - Watchdog on Pi and Arduino; stop if stale command or serial loss.
    - E-stop topic `/e_stop` (`std_msgs/Bool`) to force zero output.
- **Optional:**  
    Implement a `ros2_control` HardwareInterface that talks serial to the Arduino and use `diff_drive_controller` with YAML config.

---

### **Phase 3 — `bringup_pkg`**

- Create launch files to start:
    - LD20 node
    - `robot_state_publisher`
    - Control node
    - TF static transforms
    - RViz
- Parameters in YAML; remap serial to `/dev/arduino` and `/dev/lidar`.
- Add ROS2 launch tests and basic GTest for parsers.

---

### **Phase 4 — `perception_pkg`**

- *(Optional, not required early)*  
    Nav2 provides costmaps. Add simple obstacle filters or pointcloud utilities later.

---

### **Phase 5 — SLAM + Nav2 (`planning_pkg`)**

- **SLAM:**  
    Use `slam_toolbox` online mode. Configure frames: `map`, `odom`, `base_link`, `laser_frame`.
- **Nav2:**  
    - Configure AMCL (if using saved map) or continue SLAM for exploration.
    - Configure controllers and planners (`RegulatedPurePursuit` or `DWB`).
    - Set costmap params: footprint, inflation radius, obstacle layer from `/scan`.
    - Behavior Tree XML and Nav2 bringup launch.
- **Test Flow:**
    - Use `teleop_twist_keyboard` to validate motion and TF.
    - Verify map building; save and reload map.

---

### **Phase 6 — `interface_pkg`**

- RViz config preset.
- Teleop (keyboard/joystick) node.
- Simple web UI (optional).
- Diagnostics dashboard (`rqt_robot_monitor`).

---

### **Phase 7 — Calibration & Validation**

- Calibrate wheel radius/track width via straight/arc tests.
- Tune PID on Arduino (step inputs; log response).
- Validate odom drift; ensure `map→odom` behaves.

---

### **Phase 8 — Reliability & Ops**

- Systemd user services to auto-start bringup on boot.
- `rosbag2` recording service for field runs.
- **CPU & Performance:**
    - Use multi-threaded executor where needed.
    - Set QoS (`keep_last=1`, `reliability=best_effort` for `/scan` if high rate).

---

### **Phase 9 — CI & Quality**

- `colcon test`; add GTest/pytest for parsing, kinematics, odom.
- `clang-format`, `clang-tidy`; pre-commit hooks.
- GitHub Actions workflow for build/test on PR.

---

## 📚 Key Topics & Frames

| Topic/Frame         | Type/Description                        |
|---------------------|-----------------------------------------|
| `/cmd_vel`          | `geometry_msgs/Twist`                   |
| `/scan`             | `sensor_msgs/LaserScan` (`laser_frame`) |
| `/odom`             | `nav_msgs/Odometry` + TF `odom→base_link` |
| TF `map→odom`       | From SLAM                               |
| `robot_description` | URDF via `robot_state_publisher`        |

---

Happy building! 🚗🤖
