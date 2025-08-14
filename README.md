# GELLO for Anything Low Cost

<p align="center">
  <img src="https://drive.google.com/file/d/17b6ayigQQS0gNm5r6YZi-RezE-FZiyVT/view?usp=sharing" alt="xArm Teleop" width="30%"/>
  <img src="https://drive.google.com/file/d/1IwHOnPZi9Xd7XtmHmIMoSiWskPp-wLi9/view?usp=sharing" alt="Dobot Teleop" width="30%"/>
  <img src="https://drive.google.com/file/d/1Cz_ZdehFoAi3lnY-eTfKbWGRFAQslu8K/view?usp=sharing" alt="ARX Teleop" width="30%"/>
</p>

## Overview
This repository hosts a low-cost hardware teleoperation system capable of controlling most commercial robotic arms through three interchangeable hardware configurations.  
It targets **Ubuntu 20.04 with ROS Noetic** and will soon support teleoperation experiments inside the **SAPIEN** simulation environment *(coming soon)*.

## Features
- **Three teleop hardware configs** to fit different form factors and budgets
- **ROS-based**: teleop arm publishes servo angles to `/servo_angles`
- **Follower-arm examples** for Dobot and xArm
- **SAPIEN simulation integration** on the roadmap *(coming soon)*

---

## Installation

### 1. Install Python dependencies
```bash
pip install -r requirements.txt
```

### 2. Build the catkin workspace
```bash
catkin_make
source devel/setup.bash
```

---

## Hardware Assembly
Detailed build instructions (parts list, wiring, mechanical assembly) will be available soon:  
[Google Drive link – coming soon]

---

## Quick Start

### 1. Verify teleop arm output
```bash
roscore             # in a new terminal
rosrun uarm servo_zero.py
```
This prints real-time angles from all servos.

---

### 2. Publish teleop arm data
```bash
rosrun uarm servo_reader.py
```
The teleop arm now acts as a **ROS publisher** on the `/servo_angles` topic.

---

### 3. Control your follower arm
1. Wrap your robotic arm in a ROS node that subscribes to `/servo_angles`
2. Convert incoming angles to your arm’s joint commands

---

### 4. Example followers

**Dobot**
```bash
rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
```

**xArm**
```bash
rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
```

---

## Supported Teleop Configurations
| Teleop Config | Compatible Robot Arms |
|---------------|-----------------------|
| Config A      |                       |
| Config B      |                       |
| Config C      |                       |

*(Fill the table with your own mappings of teleop configurations to supported arm brands.)*

---

## SAPIEN Simulation *(Coming Soon)*
Upcoming release will provide a SAPIEN environment mirroring the physical teleop setup, allowing rapid prototyping and testing in simulation.

---

## Contributing
Feel free to open issues or submit pull requests for bug fixes, new follower-arm modules, or documentation improvements.

---

## License
To be determined.
