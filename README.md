# 🤖 Lerobot Everything

[![en](https://img.shields.io/badge/lang-en-blue.svg)](README.md)
[![中文](https://img.shields.io/badge/lang-中文-brown.svg)](README_CN.md)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://www.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://ubuntu.com/)
[![Apache License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)



<p align="center">
  <img src="pics/Xarm.gif" width="30%" alt="xArm Demo" />
  <img src="pics/Dobot.gif" width="30%" alt="Dobot Demo" />
  <img src="pics/Arx.gif" width="30%" alt="ARX Demo" />
</p>

---


>**🚀 Bringing Leader-Follower teleoperation system to every real robot and robot arm -- Cheaper, Smoother, Plug-and-Play**  
**💵 Starts from $60 cost!! Then controls any robot arm system!!**

*Built upon the giants: [LeRobot](https://github.com/huggingface/lerobot), [SO-100/SO-101](https://github.com/TheRobotStudio/SO-ARM100), [XLeRobot](https://github.com/Vector-Wangel/XLeRobot#)*



# 📰 News 


- 2025-08-15: **LeRobot Everything 0.1.0** hardware setup, the 1st version fully capable for three major robot arm configurations in ROS1, starts from 60$. 

---

# 📋 Table of Contents

- [Overview](#-overview)
- [✨ Features](#-features)
- [💵 Total Cost](#-total-cost-)
- [🤖 Supported Robots (find your robot in the list!)](#-supported-robots)
- [🚀 Quick Start](#-quick-start)
- [🔮 Roadmap](#-roadmap)
- [🤝 Contributing](#-contributing)

---

## 🎯 Overview

LeRobot Everything is a **low-cost, universal, leader-follower teleoperation system** for any commercial robot arms and humanoid robots through four interchangeable hardware configurations. Designed for researchers, educators, and robotics enthusiasts, it provides a standardized interface for diverse robot platforms. This project focus on extending the Lerobot to control any real robot in both real scene and simulation.

### 🎯 Target Environment (Docker coming soon)
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Simulation**: SAPIEN integration (coming soon)

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🔄 **Universal Compatibility** | Four teleop configurations covering **most (95%) commercial robot arms** |
| 📡 **ROS Integration** | Native ROS1 support with `/servo_angles` topic publishing |
| 🎮 **Real-time Control** | Low-latency servo angle transmission |
| 🔌 **Plug & Play** | Easy follower-arm integration with provided examples |
| 🛠️ **Extensible** | Simple API for adding new robot support |
| 💰 **Cost-effective** | Ultra low-cost hardware solution |
| 🎯 **Optimized Hardware** | Move smoothly and flexibly |

### 🎮 Ready-to-Use Examples

**Real Robot Examples:**
- **Dobot CR5** - Complete teleoperation setup
- **xArm Series** - Full ROS integration  
- **ARX5** - ROS-free control example

**Simulation Examples:**
- Coming soon

---

## 💵 Total Cost 💵

> [!NOTE] 
> Cost excludes 3D printing, tools, shipping, and taxes.

| Price | US | EU | CN |
| --- | --- | --- | --- |
| **Basic** (use your laptop) | **~$60** | **~€60** | **~¥360** |
| ↑ Servos | +$60 | +€60 | +¥399 |

---

## 🤖 Supported Robots (find your robot in the list!)

| Configuration | Compatible Robot Arms | Status |
|---------------|----------------------|---------|
| [**Config 1**](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation/tree/main/mechanical/Config1_STL) | Xarm7, Fanuc LR Mate 200iD, Trossen ALOHA, Agile PiPER, Realman RM65B, KUKA LBR iiSY Cobot | ✅ Ready |
| [**Config 2**](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation/tree/main/mechanical/Config2_STL) | Dobot CR5, UR5, ARX R5*, AUBO i5, JAKA Zu7 | ✅ Ready |
| [**Config 3**](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation/tree/main/mechanical/Config3_STL) | Franka FR3, Franka Emika Panda, Flexiv Rizon, Realman RM75B | ✅ Ready |

> 💡 **Need support for a different robot?** Check our [Contributing](#-contributing) section!

---
## 🚀 Quick Start


> [!NOTE] 
> If you are totally new to programming, please spend at least a day to get yourself familiar with basic Python, Ubuntu and GitHub (with the help of Google and AI). At least you should know how to set up Ubuntu system, git clone, pip install, use interpreters (VS Code, Cursor, PyCharm, etc.) and directly run commands in the terminals.

1. 💵 **Buy your parts**: [Bill of Materials](https://docs.google.com/document/d/1TjhJOeJXsD5kmoYF-kuWfPju6WSUeSnivJiU7TH4vWs/edit?tab=t.0#heading=h.k991lzlarfb8)
2. 🖨️ **Print your stuff**: [3D printing](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation/tree/main/mechanical)
3. 🔨 [**Assemble**!](Coming Soon)
4. 💻 **Software**: [Get your robot moving!](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation/blob/main/howtoplay.md)
   
For detailed hardware guide, check  [Hardware Guide](https://docs.google.com/document/d/1TjhJOeJXsD5kmoYF-kuWfPju6WSUeSnivJiU7TH4vWs/edit?tab=t.0#heading=h.k991lzlarfb8)

<!-- ---

## ⚙️ Hardware Assembly

> 📚 **Detailed build instructions coming soon!**

We're preparing comprehensive documentation including:
- 📋 Complete parts list
- 🔌 Wiring diagrams
- 🔧 Mechanical assembly guide
- 🎥 Video tutorials

**Stay tuned for the Google Drive link with full documentation!** -->

---

## 🔮 Roadmap

### 🎯 Coming Soon
- [ ] **SAPIEN Simulation Environment**: Install and Play!
  - Virtual teleop setup mirroring physical hardware
  - Rapid prototyping and testing capabilities
  - Integration with existing SAPIEN workflows

### 🚀 Future Features
- [ ] **ROS2 Support**
- [ ] **Docker Image**
- [ ] **Humanoid System: Config4**

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### 💡 Feature Requests

### 🔧 Code Contributions

### 🤖 Adding New Robot Support

---

## 👥 Main Contributors

- **Yanwen Zou** - 
- **?** - 
- **Junda Huang** - 
- **Gaotian Wang** - 

This project builds upon the excellent work of:
- [LeRobot](https://github.com/huggingface/lerobot) - The foundation for robot learning
- [SO-100/SO-101](https://github.com/TheRobotStudio/SO-ARM100) - Hardware inspiration
- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) - Extended robot support

Thanks to all the talented contributors behind these detailed and professional projects!

---

<div align="center">

**Made with ❤️ for the robotics community**

[![GitHub stars](https://img.shields.io/github/stars/yanwen-zou/UArm-Cross-Embodiment-Teleoperation?style=social)](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation)
[![GitHub forks](https://img.shields.io/github/forks/yanwen-zou/UArm-Cross-Embodiment-Teleoperation?style=social)](https://github.com/yanwen-zou/UArm-Cross-Embodiment-Teleoperation)

</div>
