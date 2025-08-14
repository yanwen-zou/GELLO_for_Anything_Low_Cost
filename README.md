# UArm: Ultra Low-Cost General Interface for Robot Manipulation 



<p align="center">
  <img src="pics/Xarm.gif" width="30%" />
  <img src="pics/Dobot.gif" width="30%" />
  <img src="pics/Arx.gif" width="30%" />
</p>

## Overview
This repository hosts a low-cost hardware teleoperation system capable of controlling most commercial robotic arms through three interchangeable hardware configurations.  
It targets **Ubuntu 20.04 with ROS Noetic** and will soon support teleoperation experiments inside the **SAPIEN** simulation environment *(coming soon)*.

## Features
- **Three teleop hardware configs** to cover **almost all commercial robot arms**
- **ROS1-based**: teleop arm publishes servo angles to `/servo_angles`
- **Follower-arm examples** for Dobot, xArm. We also provide ARX5 control example with ROS-free version.
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

When the hardware is ready, you can run the following code to test UArm:

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
| Config 1      | Xarm7, Fanuc LR Mate 200iD, Trossen ALOHA, Agile PiPER, Realman RM65B, KUKA LBR iiSY Cobot |
| Config 2      | Dobot CR5, UR5, ARX R5*, AUBO i5, JAKA Zu7 |
| Config 3      | Franka FR3, Franka Emika Panda, Flexiv Rizon, Realman RM75B |


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
