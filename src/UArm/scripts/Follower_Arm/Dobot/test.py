#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：用于测试 Bestman_Real_CR5 机器人的 get_current_states 功能
"""
import re 
import time
from api import Bestman_Real_CR5

def main():
    # 机器人IP地址 - 请根据实际情况修改
    robot_ip = "192.168.5.1"  # 请替换为实际的机器人IP地址
    
    try:
        print("正在连接机器人...")
        # 实例化 Bestman_Real_CR5 类
        robot = Bestman_Real_CR5(ip=robot_ip, text_log=True)
        print("机器人连接成功！")
        
        # 等待反馈数据初始化
        print("等待反馈数据初始化...")
        time.sleep(2)
        
        # 获取当前状态
        print("\n正在获取机器人当前状态...")
        current_states = robot.get_current_states()
        
        # 检查 current_states 的类型
        # print(f"\n=== current_states 类型信息 ===")
        # print(f"类型: {type(current_states)}")
        # print(f"数据类型: {current_states.dtype}")
        # print(f"形状: {current_states.shape}")
        # print(f"字段名称: {current_states.dtype.names}")
        
        print(f"\n=== current_states 内容 ===")
        print(current_states)
        
        print("\n=== 机器人当前状态信息 ===")
        print(f"机器人模式: {current_states['RobotMode']}")
        print(f"时间戳: {current_states['TimeStamp']}")
        print(f"运行时间: {current_states['RunTime']}")
        print(f"速度缩放: {current_states['SpeedScaling']}")
        
        # 获取关节角度
        joint_angles = robot.get_current_joint_angles()
        print(f"\n当前关节角度 (度):")
        for i, angle in enumerate(joint_angles, 1):
            print(f"  J{i}: {angle:.3f}°")
        
        # 获取关节速度
        joint_velocities = robot.get_current_joint_velocities()
        print(f"\n当前关节速度 (度/秒):")
        for i, velocity in enumerate(joint_velocities, 1):
            print(f"  J{i}: {velocity:.3f}°/s")
        
        # 获取TCP位姿
        tcp_pose = robot.get_current_end_effector_pose()
        print(f"\n当前TCP位姿:")
        print(f"  位置 (mm): X={tcp_pose[0]:.3f}, Y={tcp_pose[1]:.3f}, Z={tcp_pose[2]:.3f}")
        print(f"  姿态 (度): Roll={tcp_pose[3]:.3f}, Pitch={tcp_pose[4]:.3f}, Yaw={tcp_pose[5]:.3f}")
        
        # 获取TCP速度
        tcp_speed = robot.get_current_tcp_speed()
        print(f"\n当前TCP速度:")
        print(f"  线速度 (mm/s): X={tcp_speed[0]:.3f}, Y={tcp_speed[1]:.3f}, Z={tcp_speed[2]:.3f}")
        print(f"  角速度 (度/秒): Roll={tcp_speed[3]:.3f}, Pitch={tcp_speed[4]:.3f}, Yaw={tcp_speed[5]:.3f}")
        # Move end effector to current position with z + 100mm
        
        print(f"\n=== 移动机器人 ===")
        print("正在移动机器人到当前位置x方100mm...")
        target_pose = [tcp_pose[0]+ 100, tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]]
        print(f"目标位姿: X={target_pose[0]:.3f}, Y={target_pose[1]:.3f}, Z={target_pose[2]:.3f}")
        print(f"目标姿态: Roll={target_pose[3]:.3f}, Pitch={target_pose[4]:.3f}, Yaw={target_pose[5]:.3f}")
        

        # robot.SpeedFactor(50)
        # robot.stop()
        # robot.Continue()
        st = robot.get_current_states()
        # print("PauseCmdFlag =", st['PauseCmdFlag'],
        #     "SpeedScaling =", st['SpeedScaling'])

        joint_angles = joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5]-50
        try:

            robot.move_end_effector_to_goal_pose(target_pose, speed_ratio=50, accel_ratio=30, wait=True)
            time.sleep(3)
            robot.move_arm_to_joint_angles(joint_angles, speed_ratio=50, accel_ratio=30, wait=True)
            print("机器人移动完成！")

            # 获取移动后的TCP位姿
            new_tcp_pose = robot.get_current_end_effector_pose()
            print(f"\n移动后TCP位姿:")
            print(f"  位置 (mm): X={new_tcp_pose[0]:.3f}, Y={new_tcp_pose[1]:.3f}, Z={new_tcp_pose[2]:.3f}")
            print(f"  姿态 (度): Roll={new_tcp_pose[3]:.3f}, Pitch={new_tcp_pose[4]:.3f}, Yaw={new_tcp_pose[5]:.3f}")
            
        except Exception as e:
            print(f"机器人移动失败: {e}")
        
    except Exception as e:
        print(f"错误: {e}")
        print("请检查:")
        print("1. 机器人IP地址是否正确")
        print("2. 机器人是否已上电并连接到网络")
        print("3. 网络连接是否正常")
    
    finally:
        # 关闭连接
        try:
            robot.ModbusClose(0)
            robot.close()
            print("机器人连接已关闭")
        except:
            pass

if __name__ == "__main__":
    main()
