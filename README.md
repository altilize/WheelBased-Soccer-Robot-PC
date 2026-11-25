# Wheel-Based Soccer Robot: PC Vision and Strategy

C++/OpenCV program for a KRSBI Beroda soccer robot: dual-camera vision, game behaviors, STM32 serial link and basestation UDP.

<!-- TODO: add demo GIF/photo -->

## Overview

This is the high-level controller of R2C's wheeled soccer robot (codename *Warrior*) for **KRSBI-B** (Kontes Robot Sepak Bola Indonesia Beroda). An onboard Linux PC reads two USB cameras, detects the ball, goal, teammates and opponents, decides what the robot should do, and sends motion commands to the STM32 motion controller ([WheelBased-Soccer-Robot-Microcontroller](https://github.com/altilize/WheelBased-Soccer-Robot-Microcontroller)). A basestation on the team network selects skills and can drive the robot manually over UDP multicast.

## Features

- **Dual-camera vision** (640x480, V4L2)
  - Omnidirectional camera: frame shift, circular mask and rotation to align with the robot heading, then HSV segmentation of field, ball, teammates and opponents. Field boundary from a convex hull.
  - Front camera: goal, teammate and opponent detection, plus a 64-ray scan for dark obstacles.
- **Ball tracking**: contour plus `minEnclosingCircle` detection, scalar Kalman filters on ball, teammate, opponent and goal coordinates, and a ball direction predictor used to compute an intercept heading.
- **Behaviors** selected from the basestation: chase ball, shoot at goal, pass to teammate, track stationary or moving ball, fast chase, soft and hard kick.
- **Goalkeeper entrance**: autonomous field entry by driving through grid waypoints (0,2) → (8,2) → (8,0), then idle and left/right guarding.
- **Grid navigation**: move to a field grid cell using odometry from the STM32, with far/near speed profiles.
- **PID heading control** with integral anti-windup for aiming at the ball.
- **Manual override** from the basestation (forward, back, left, right, rotate, stop).
- **Logging** of grid position, compass and state transitions to dated files in `logging/`.
- **GTK calibration tools** for camera parameters (`kamera`) and HSV thresholds (`segmentasi`).

## Architecture

```
 Omni camera --+
               |   thread_citra: segmentation, detection, behaviors
 Front camera -+-----------------------------+
                                             v
                                        pcData (motion cmd)
                                             |
 Basestation <--UDP multicast--> receive/send threads
 (skills, manual,  224.16.32.110:12478       |
  grid reset)                                v
                              thread_komunikasi: USB serial 115200
                                             |
                                             v
                         STM32 --> holonomic kinematics --> 4 omni-wheel motors
                           |
                           '--> compass, grid, X/Y, IR, keypad back to PC
```

The program runs four POSIX threads:

| Thread | Role |
| --- | --- |
| `thread_citra` | Grab frames from both cameras, run vision and behavior logic, fill the outgoing command |
| `thread_komunikasi` | Send the 13-byte command to the STM32 on `/dev/ttyACM0` and parse the 16-byte status reply |
| `receiveBasestation` | Join the multicast group and decode skill, manual and grid/compass reset packets |
| `sendBasestation` | Broadcast compass, X/Y position, ball angle, kick readiness and active skill every 50 ms |

### STM32 link

Commands carry a motion enable flag, 16-bit speed, signed heading (0° forward, 90° right), signed 16-bit rotation `vz`, handler and kick fields. A second packet type sets the grid cell and compass offset. Every packet ends with a checksum (bitwise NOT of the byte sum). The STM32 converts speed and heading into holonomic wheel speeds:

```
vx = speed * sin(heading)
vy = speed * cos(heading)
```

## Hardware and Software Stack

| Item | Details |
| --- | --- |
| Platform | Linux PC on the robot |
| Cameras | 2x USB UVC webcams, addressed by `/dev/v4l/by-id/` path |
| Motion controller | STM32F407 over USB CDC serial |
| Language | C++14 / C++17 (GUI library) |
| Libraries | OpenCV 4, gtkmm-3.0, pthreads |
| Build | CMake ≥ 3.21 |
| Tools | `v4l2-ctl` (v4l-utils) for camera parameters |

## Repository Structure

```
├── main.cpp                 # entry point, threads, behavior state machine, logging
├── kamera.cpp               # GTK tool: choose a camera and tune/save V4L2 parameters
├── segmentasi.cpp           # GTK tool: HSV threshold calibration on live or saved frames
├── CMakeLists.txt
├── library/
│   ├── Citra.*              # image processing and object detection
│   ├── Algoritma.*          # behaviors (chase, pass, shoot, catch) and PID
│   ├── Motion.*             # grid navigation, rotation, obstacle avoidance helpers
│   ├── Camera.*             # camera discovery and V4L2 parameter load/save
│   ├── KomunikasiMikro.*    # STM32 packet encode/decode
│   ├── InisiasiSerial.*     # termios serial setup
│   ├── UDPListen.* UDPSend.*# basestation multicast
│   ├── TypeData.h           # shared structs (pcData, STM32Data, BasestationData)
│   └── BackCamera.h, StrikerCamera.h  # per-robot camera paths and omni frame offsets
├── gui/                     # GTK widgets and Glade layouts for the calibration tools
├── ui/                      # Glade layouts
├── kalibrasi/               # HSV threshold files read at startup, legacy calibration utilities
├── datacamera/              # saved V4L2 parameters per camera
├── data/                    # sample frames for offline segmentation
├── logging/                 # runtime logs (ignored by git)
└── setcam.sh                # apply V4L2 settings for a given /dev/videoN
```

## Build and Run

Dependencies (Ubuntu):

```bash
sudo apt install build-essential cmake pkg-config libopencv-dev libgtkmm-3.0-dev v4l-utils
```

Build out of source:

```bash
cmake -S . -B build
cmake --build build -j
```

Run from the repository root, because calibration files are loaded from relative paths (`kalibrasi/`, `datacamera/`, `logging/`):

```bash
./setcam.sh 0              # optional: apply camera settings to /dev/video0
./build/kamera             # tune and save camera parameters
./build/segmentasi         # calibrate HSV thresholds
./build/main               # run the robot
```

Before running `main`:

- Set the camera paths in `library/BackCamera.h` to match your `/dev/v4l/by-id/` devices.
- Connect the STM32 on `/dev/ttyACM0` and make sure your user is in the `dialout` group.
- The basestation must be on the same network and use multicast group `224.16.32.110`, port `12478`.

## My Role

This is shared R2C team code. My part (Moses Jaguar Ersaptiarso) focused on motion and integration: the STM32 serial protocol and PC to microcontroller data flow, grid navigation and the goalkeeper entrance sequence, sensor data integration (compass, odometry grid, IR, ToF flag), and runtime logging.
