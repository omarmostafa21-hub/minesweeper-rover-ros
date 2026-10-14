# 6WD Minesweeper Rover — ROS & Embedded Control System

An autonomous and teleoperated 6WD Minesweeper rover running on ROS Noetic, integrated with Arduino Mega 2560, Cytron motor drivers, planetary geared encoders, and an MPU-6500 IMU for hybrid sensor fusion odometry.

## System Specifications
- **Chassis:** 6WD Rocker-Bogie / Skid-Steer chassis
- **Wheel Diameter:** 0.10 m (10 cm)
- **Wheel Track (Base):** 0.52 m (52 cm)
- **Motors:** PG45ZY Planetary Geared DC Motors (Reduction Ratio 19.2:1)
- **Motor Drivers:** Cytron Motor Drivers (PWM + DIR control)
- **Sensors:** 
  - 6x Magnetic Hall Effect Encoders (Calibrated at 456 ticks/rev)
  - MPU-6500 6-DOF IMU (I2C Address: `0x69`)
- **Controller:** Arduino Mega 2560 connected via `rosserial`

## Key Architecture & Features
- **Low-Level Motor Control:** Smooth acceleration limiting (Slew-rate filter) to protect gearboxes and prevent voltage drops.
- **Robust Hardware Interfacing:** Noise-tolerant state-change polling for encoders and hardware timeout-protected I2C bus preventing communication deadlocks.
- **Hybrid Sensor Fusion:** Decouples linear displacement (fused encoder ticks) from robot orientation (drift-filtered IMU yaw integration).
- **Control Interface:** PS4 wireless controller teleoperation mapped to `/cmd_vel`.
- **Live Visualization:** Real-time 19x19 arena coordinate tracking GUI.

## Running the Project

# 1. Attach Arduino to WSL2 (from Windows PowerShell)
usbipd attach --wsl --busid <BUSID>

# 2. Launch Serial Bridge (inside WSL)
sudo chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

# 3. Launch Core Navigation & Teleop
roslaunch minesweeper_core arena_live.launch
