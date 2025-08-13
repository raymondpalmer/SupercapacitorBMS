# bms_can_bridge_cpp (ROS 2 Humble, C++)

C++ node that decodes your custom BMS CAN frames (CAN 2.0A/B, **250 kbps**) into ROS 2 topics.
It expects raw frames from `ros2_socketcan` on `/from_can`.

## Build & Run
```bash
# Bring up SocketCAN @ 250 kbps
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 250000

# Build
colcon build --packages-select bms_can_bridge_cpp
. install/setup.bash

# Launch (starts socketcan receiver + decoder)
ros2 launch bms_can_bridge_cpp bms_can_bridge.launch.py interface:=can0
```

## Topics
- Input: `/from_can` (`can_msgs/msg/Frame`)
- Output:
  - `/bms/batt_st1` (`sensor_msgs/msg/BatteryState`)
  - `/bms/alarms`, `/bms/faults`, `/bms/limits`, `/bms/info` (`diagnostic_msgs/msg/DiagnosticArray`)
  - `/bms/switches` (`std_msgs/msg/UInt32`)
  - `/bms/charge_request`, `/bms/ctrl_info` (`diagnostic_msgs/msg/DiagnosticArray`)
  - `/bms/cell/volt_summary`, `/bms/cell/temp_summary` (`std_msgs/msg/Float32MultiArray`)
  - `/bms/cell/index_min|max` (`std_msgs/msg/UInt16`)
  - `/bms/cell/voltages`, `/bms/cell/temps` (`std_msgs/msg/Float32MultiArray`)
  - `/bms/rated_capacity_ah`, `/bms/nominal_voltage_v` (`std_msgs/msg/Float32`)

## Parameters
- `from_topic` (default `/from_can`)
- `max_cells` (default `128`)
- `publish_full_arrays_hz` (default `2.0`)
