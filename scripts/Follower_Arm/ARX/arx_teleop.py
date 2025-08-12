#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import serial
import time
import re
import numpy as np
import os
import sys

# ====== ARX5 接口路径配置 ======
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(ROOT_DIR))
import arx5_interface as arx5  # noqa


# ====== 主臂串口读取类 ======
class ServoReader:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.SERIAL_PORT = port
        self.BAUDRATE = baudrate
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        print(f"[ServoReader] 串口 {self.SERIAL_PORT} 已打开")

        self.zero_angles = [0.0] * 7
        self.current_angles = [0.0] * 7
        self.lock = threading.Lock()

        self._init_servos()

    def send_command(self, cmd):
        self.ser.write(cmd.encode('ascii'))
        time.sleep(0.008)
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!')
        for i in range(7):
            self.send_command("#000PCSK!")
            self.send_command(f'#{i:03d}PULK!')
            response = self.send_command(f'#{i:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[i] = angle if angle is not None else 0.0
        print("[ServoReader] 舵机初始角度校准完成")

    def read_loop(self, hz=100):
        dt = 1.0 / hz
        while True:
            new_angles = [0.0] * 7
            for i in range(7):
                response = self.send_command(f'#{i:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    new_angles[i] = angle - self.zero_angles[i]
            with self.lock:
                self.current_angles = new_angles
            time.sleep(dt)

    def get_angles(self):
        with self.lock:
            return list(self.current_angles)


# ====== 从臂控制类（加入速度限制） ======
class ArxTeleop:
    def __init__(self, model="X5", interface="can0"):
        self.ctrl = arx5.Arx5JointController(model, interface)
        self.robot_cfg = self.ctrl.get_robot_config()
        self.ctrl_cfg = self.ctrl.get_controller_config()
        self.dof = self.robot_cfg.joint_dof  # 例如 6

        # PID 增益
        gain = arx5.Gain(self.dof)
        gain.kd()[:] = 0.01
        self.ctrl.set_gain(gain)

        self.ctrl.reset_to_home()

        # 主从映射参数（假设 servo 0-5 对应关节 0-5）
        self.scale = [1.0] * self.dof
        self.offset_rad = [0.0] * self.dof

        # 夹爪通道
        self.gripper_index = 6
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30

        # === 速度限制参数（可按关节分别调整） ===
        # 关节最大速度（rad/s），默认统一上限（约 69 deg/s）
        self.max_joint_vel = np.array([1.2] * self.dof, dtype=np.float64)
        self.max_joint_vel[3:] = np.array([2] * (self.dof - 3), dtype=np.float64)
        # 夹爪最大“归一化速度” (/s)，0~1 区间每秒最大变化
        self.max_gripper_vel = 2.0

        # 可选：速度前馈开关与比例（参考你的示例）
        self.use_vel_feedforward = False
        self.vel_ff_gain = 0.3

        # 限速状态（上一周期已下发的目标）
        self._inited_cmd = False
        self._last_cmd_pos = np.zeros(self.dof, dtype=np.float64)
        self._last_cmd_grip = 0.0

    def _deg_to_rad_mapped(self, master_angles_deg):
        """主臂角度(度) -> 从臂关节角(弧度)"""
        joints_rad = np.zeros(self.dof, dtype=np.float64)
        for j in range(self.dof):
            joints_rad[j] = np.deg2rad(master_angles_deg[j]) * self.scale[j] + self.offset_rad[j]
        
        joints_rad[4], joints_rad[5] = -joints_rad[5], joints_rad[4] #swap j5 and j6
        return joints_rad

    def _map_gripper(self, master_angles_deg):
        grip_deg = master_angles_deg[self.gripper_index]
        grip_norm = (grip_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
        return float(np.clip(grip_norm, 0.0, 1.0))

    def send_cmd(self, master_angles_deg):
        # 期望位姿
        desired_pos = self._deg_to_rad_mapped(master_angles_deg)
        desired_grip = self._map_gripper(master_angles_deg)

        dt = float(self.ctrl_cfg.controller_dt)

        # 初始化：第一帧直接对齐
        if not self._inited_cmd:
            self._last_cmd_pos[:] = desired_pos
            self._last_cmd_grip = desired_grip
            self._inited_cmd = True

        # === 关节限速：按周期限制最大步进 ===
        max_step = self.max_joint_vel * dt                     # 每周期最大允许位移
        delta = desired_pos - self._last_cmd_pos
        delta_clipped = np.clip(delta, -max_step, max_step)
        cmd_pos = self._last_cmd_pos + delta_clipped

        # === 夹爪限速 ===
        grip_delta = desired_grip - self._last_cmd_grip
        grip_step = self.max_gripper_vel * dt
        grip_cmd = self._last_cmd_grip + float(np.clip(grip_delta, -grip_step, grip_step))

        # 组织命令
        js = arx5.JointState(self.dof)

        js.pos()[:] = cmd_pos
        js.gripper_pos = grip_cmd

        # 可选：速度前馈（基于限速后的位移）
        if self.use_vel_feedforward and hasattr(js, "vel"):
            safe_vel = delta_clipped / max(dt, 1e-6)
            js.vel()[:] = self.vel_ff_gain * safe_vel

        # 下发
        self.ctrl.set_joint_cmd(js)

        # 更新状态
        self._last_cmd_pos[:] = cmd_pos
        self._last_cmd_grip = grip_cmd


# ====== 主程序 ======
if __name__ == "__main__":
    try:
        # 创建主臂读取器
        servo_reader = ServoReader(port="/dev/ttyUSB0", baudrate=115200)

        # 创建从臂控制器（带速度限制）
        teleop = ArxTeleop(model="X5", interface="can0")

        # 启动读取线程
        t_reader = threading.Thread(target=servo_reader.read_loop, kwargs={"hz": 100}, daemon=True)
        t_reader.start()

        print("[Main] 遥操作开始，按 Ctrl+C 退出。")
        dt = teleop.ctrl_cfg.controller_dt
        while True:
            master_angles = servo_reader.get_angles()
            teleop.send_cmd(master_angles)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[Main] 中断，机械臂回到 home...")
        teleop.ctrl.reset_to_home()
        print("[Main] 已退出。")
