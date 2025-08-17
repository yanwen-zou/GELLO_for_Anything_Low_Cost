
# 🔧 System Setup

### Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

### Step-by-Step Setup

1. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

2. **Build Catkin Workspace**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

3. **Verify Installation**
   ```bash
   # Test if ROS can find the package
   rospack find uarm
   ```

---
# Plug-and-Play in Real or Simulation

### ✅ Verify Teleop Arm Output
```bash
# Terminal 1: Start ROS core
roscore

# Terminal 2: Test servo readings
rosrun uarm servo_zero.py
```
This displays real-time angles from all servos.

### ✅ Publish Teleop Data
```bash
# Terminal 3: Start teleop publisher
rosrun uarm servo_reader.py
```
Your teleop arm now publishes to `/servo_angles` topic.

### ✅ Control Follower Arm
```bash
# Terminal 4: Choose your robot

# For Dobot CR5
rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py

# For xArm
rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
```

---
