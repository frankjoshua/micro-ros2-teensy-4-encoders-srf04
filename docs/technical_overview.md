---
# Technical Overview: micro-ros2-teensy-4-encoders-srf04

## Project Purpose
This project provides firmware for a differential-drive robot using a Teensy 4.x microcontroller, quadrature encoders, and micro-ROS 2 for ROS integration. It is designed for extensibility, modularity, and robust ROS 2 communication using PlatformIO.

## System Architecture
**Main Data Flow:**
1. **Encoders** → Read wheel positions and speeds
2. **Kinematics** → Convert encoder data to robot velocities
3. **PID** → Compute motor commands to achieve target velocities
4. **Motor** → Send commands to motor drivers
5. **Odometry** → Track robot pose (x, y, θ)
6. **micro-ROS** → Publish odometry, subscribe to velocity commands

**Pin Mapping (default, see `main.cpp`):**
- Left Encoder: Pin 6
- Right Encoder: Pin 5
- Left Motor: Motor(2, 0) (see Motor.cpp for mapping)
- Right Motor: Motor(1, 1)
- Serial2: Used for motor commands

## Directory Structure
- `src/main.cpp`: Main application logic and ROS integration
- `lib/encoder/`: Quadrature encoder interface (uses Encoder_Buffer)
- `lib/kinematics/`: Kinematics calculations for differential drive
- `lib/motor/`: Motor driver abstraction (serial-based)
- `lib/odometry/`: Odometry computation (pose tracking)
- `lib/pid/`: PID controller for closed-loop control
- `include/`: Shared headers
- `docs/`: Documentation
- `media/`: Images/media
# Technical Overview: micro-ros2-teensy-4-encoders-srf04

## Purpose
Firmware for a differential-drive robot based on Teensy 4.x, using quadrature encoders for feedback and micro-ROS for ROS 2 integration. Built with PlatformIO and structured for easy tuning and extension.

## High-level data flow
1. Encoders read wheel positions and compute per-wheel RPM
2. Kinematics converts target linear/angular velocities to target wheel RPM
3. PID controllers compare target vs measured RPM and produce motor deltas
4. Motor driver maps deltas to Serial2 bytes for the external motor controller
5. Odometry integrates velocities for pose; micro-ROS publishes telemetry and receives commands

## Pin mapping and interfaces (defaults from `src/main.cpp`)
- Encoder chip-select: Left = 6, Right = 5 (Encoder_Buffer over SPI)
- Motor outputs: Serial2 byte protocol
   - Left motor bytes: 1–127
   - Right motor bytes: 128–255
- Motor instances: `Motor leftMotor(2, 0)`, `Motor rightMotor(1, 1)` (direction flag is used to pick byte range)

## Repository layout
- `src/main.cpp` — main loop, micro-ROS node, control loop
- `lib/encoder/` — encoder interface and RPM calculation
- `lib/kinematics/` — wheel/robot velocity conversions
- `lib/pid/` — simple PID controller
- `lib/motor/` — Serial2 mapping to motor controller
- `lib/odometry/` — basic dead-reckoning
- `platformio.ini` — build/board settings and library deps

## Key parameters (tune for your robot in `main.cpp`)
- MAX_RPM: 80 (example)
- WHEEL_DIAMETER: 0.15 m
- LR_WHEELS_DISTANCE: 0.35 m (track width)
- TICKS_PER_REVOLUTION: 130000 (depends on encoder and mode)
- PID gains: K_P=0.009, K_I=0.0, K_D=0.0, limits = ±2048

## Control loop timing
- Encoder read + PID + publish: every ~50 ms
- micro-ROS executor spin: ~2 ms budget each loop
- Connection watchdog: ping agent every 10 s; on failure, blink LED13 and auto-reboot after ~2 s
- Command timeout: if no `cmd_vel` for >400 ms, targets set to zero (stop)

## ROS 2 topics
- Subscribes: `cmd_vel` (geometry_msgs/Twist)
- Publishes: `vel` (geometry_msgs/Twist) — for telemetry/debug only
   - linear.x: computed forward velocity (m/s)
   - angular.z: computed yaw rate (rad/s)
   - linear.z: raw left encoder count (for debugging)
   - linear.y: raw right encoder count (for debugging)
   - angular.x: left wheel RPM (measured)
   - angular.y: last left motor delta (PID output)

Note: nav_msgs/Odometry and TF are not published yet, though placeholders exist.

## Encoder details
- Uses Encoder_Buffer with separate chip-select lines (SPI shared)
- RPM formula: `rpm = (delta_counts / ticks_per_rev) * (60000 / delta_time_ms)`
- Left encoder reading is negated in code to align signs: forward motion should yield the same sign on both wheels. If your hardware already matches, remove the negation.

## Motor mapping details
- `Motor::adjust(delta)` accumulates a `power` value and maps it to a byte:
   - Left: map [-1..1] → [1..127]
   - Right: map [-1..1] → [128..255]
- The current code does not clamp `power` and the PID outputs are in the range ±2048. Without normalization, bytes can overflow or wrap. See “Uneven wheel speeds” for mitigation.

## Build and flash
1. Install PlatformIO
2. Select the right board in `platformio.ini` (default: teensy40)
3. Build and upload from VS Code or CLI

CLI (optional):
```bash
# build and upload
pio run -t upload

# monitor serial output
pio device monitor -b 115200
```

## Tuning and calibration
- Ticks per revolution (TPR): Measure for your encoder (include x4 if used).
- Geometry: Measure wheel diameter and track width (LR_WHEELS_DISTANCE).
- PID: Start with P only; add I to remove steady-state bias; add D to damp overshoot. Keep motor input in [-1, 1].

## Troubleshooting: one wheel runs faster
If one wheel consistently spins faster, check these in order:

1) Scaling mismatch (most likely)
- PID compute returns ±2048 by default, but `Motor::adjust` expects deltas around ±1.
- Effect: power quickly exceeds ±1; map() extrapolates beyond the 1–127 / 128–255 ranges; bytes may wrap or saturate differently per side, leading to asymmetry.
- Fix (code-level):
   - Normalize PID output before calling `Motor::adjust`, e.g. `delta = constrain(pid / PID_MAX, -1.0, 1.0)`; or set PID min/max to ±1 and adjust K values accordingly.
   - Clamp `power` internally: `power = constrain(power + delta, -1.0, 1.0)` before mapping; also clamp the mapped byte to [1,127] or [128,255].

2) Sign conventions
- In `Encoder.cpp`, the left encoder reading is negated. This is intended to make forward motion yield the same sign on both wheels.
- If your wiring/encoders already produce matching signs, that negation will invert the left wheel and confuse PID.
- Fix: ensure both measured RPMs are positive when moving forward. Remove or add negation to match your hardware.

3) PID integral bias
- K_I is 0.0, so any static bias (friction, motor mismatch) isn’t corrected. One wheel may run faster at steady state.
- Fix: add a small I term and integral windup guard. Example: K_I ≈ 0.001–0.01, monitor stability.

4) Unequal TPR, geometry, or mechanical mismatch
- If the encoders or wheel diameters differ left vs right, the same command yields different ground speeds.
- Fix: verify TICKS_PER_REVOLUTION and wheel diameters per side; if different, account for it or calibrate via a per-wheel scale factor.

5) Serial protocol assumptions
- The current implementation sends raw bytes on a shared Serial2 to control both motors, distinguished only by value ranges (1–127 vs 128–255).
- If values overflow the expected ranges, the motor controller may interpret them incorrectly (e.g., wrong channel or clipped speed), causing asymmetry.
- Fix: ensure strict clamping to the valid byte ranges and consider adding rate limiting.

Quick diagnostics
- Log `goalRPM.motor1/motor2`, `encoderData.rpm.left/right`, and the actual bytes sent to Serial2. Both measured RPMs should be positive and similar for a straight `cmd_vel`.
- Temporarily set K_P small and K_I=K_D=0; drive forward and check raw RPM parity.
- Swap encoder CS lines in software to see if the “fast side” follows the encoder or the motor channel.

## Known limitations and next steps
- Only publishes a debug Twist; add nav_msgs/Odometry and TF broadcaster.
- Add proper scaling/clamping in `Motor` and normalization in `main.cpp`.
- Add per-wheel calibration factors and encoder sign configuration.
- Consider adding safety features: max velocity limits and E-stop handling.

## Notes on micro-ROS connectivity
- If the agent disconnects, the firmware blinks LED 13 and reboots after ~2 s.
- Ensure `cmd_vel` is being published; otherwise the controller times out after 400 ms and stops.
