#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gripper_test.py
---------------
硬编码参数的 Robotiq 夹爪功能测试脚本。
保存到与 api.py 同目录后直接运行：  python3 gripper_test.py
"""

import time, sys
from api import Bestman_Real_CR5   # 确保已在 PYTHONPATH 或同目录

# -------- 固定参数 --------
ROBOT_IP  = "192.168.5.1"   # CR5 控制箱 IP
SLAVE_ID  = 9               # 夹爪默认从站 ID

# ---------------- 工具函数 ----------------
def banner(msg): print(f"\n========== {msg} ==========")

def main():
    try:
        banner("连接机器人")
        robot = Bestman_Real_CR5(ip=ROBOT_IP, text_log=True)
        print("连接成功")

        banner("激活夹爪")
        robot._ensure_gripper_modbus(slave_id=SLAVE_ID)
        print("激活成功，Modbus 句柄 =", robot._mb_idx)

        banner("位置测试 (0, 255, 128)")
        for pos in (0, 255, 128):
            print(f"→ 设定位置 {pos}")
            robot.gripper_goto(pos, wait=True)
            real = robot.get_gripper_position()
            print(f"   实际位置 {real} (Δ={abs(real-pos)})")

        banner("连续开合 3 次")
        for _ in range(3):
            robot.open_gripper(wait=True); time.sleep(0.2)
            robot.close_gripper(wait=True); time.sleep(0.2)
        print("循环完成")

        banner("读取故障码 gFLT")
        gflt = robot._rd_u16(0x07D1)[0]
        print(f"gFLT = {gflt} ({'无故障' if gflt == 0 else '请查手册'})")

    except Exception as e:
        print("✖ 测试失败:", e)

    finally:
        # 清理资源
        try:
            if hasattr(robot, "_mb_idx") and robot._mb_idx:
                robot.ModbusClose(robot._mb_idx)
            robot.sendRecvMsg("SetToolPower(0)")
            robot.close()
            print("已关闭机器人连接并断电")
        except Exception:
            pass

if __name__ == "__main__":
    main()
