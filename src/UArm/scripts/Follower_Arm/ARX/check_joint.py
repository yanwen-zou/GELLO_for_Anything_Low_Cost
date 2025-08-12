#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ARX机械臂关节角度读取脚本
用于读取ARX机械臂的起始关节角度
"""

import os
import sys
import time
import numpy as np

# 添加ARX5 SDK路径
ROOT_DIR = "/home/onestar/arx5-sdk/python"
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5


def init_arx5_controller(model="X5", interface="can1"):
    """
    初始化ARX5机械臂控制器
    
    Args:
        model (str): ARX机械臂型号 (X5 或 L5)
        interface (str): CAN总线接口名称 (如 can0)
    
    Returns:
        arx5.Arx5JointController: 初始化好的关节控制器
    """
    try:
        print(f"正在初始化ARX5机械臂 (型号: {model}, 接口: {interface})...")
        
        # 获取机器人配置
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", robot_config.joint_dof
        )
        
        # 设置多线程模式
        controller_config.background_send_recv = True
        
        # 创建ARX5关节控制器
        arx5_joint_controller = arx5.Arx5JointController(
            robot_config, controller_config, interface
        )
        
        # 设置日志级别
        np.set_printoptions(precision=3, suppress=True)
        arx5_joint_controller.set_log_level(arx5.LogLevel.INFO)
        
        print("ARX5机械臂初始化成功!")
        return arx5_joint_controller
        
    except Exception as e:
        print(f"初始化ARX5机械臂失败: {e}")
        return None


def read_joint_angles(controller, num_readings=5, delay=0.5):
    """
    读取ARX机械臂的关节角度
    
    Args:
        controller: ARX5关节控制器
        num_readings (int): 读取次数，用于验证数据稳定性
        delay (float): 每次读取之间的延迟时间（秒）
    
    Returns:
        dict: 包含关节角度信息的字典
    """
    if controller is None:
        print("控制器未初始化，无法读取关节角度")
        return None
    
    print(f"\n开始读取关节角度 (读取{num_readings}次，间隔{delay}秒)...")
    
    joint_readings = []
    
    for i in range(num_readings):
        try:
            # 获取关节状态
            joint_state = controller.get_joint_state()
            
            # 提取关节位置和速度
            joint_positions = joint_state.pos().copy()
            joint_velocities = joint_state.vel().copy()
            gripper_position = joint_state.gripper_pos
            
            # 存储读取结果
            reading = {
                'positions': joint_positions,
                'velocities': joint_velocities,
                'gripper_pos': gripper_position,
                'timestamp': time.time()
            }
            joint_readings.append(reading)
            
            print(f"第{i+1}次读取:")
            print(f"  关节位置 (rad): {joint_positions}")
            print(f"  关节位置 (deg): {np.degrees(joint_positions)}")
            print(f"  关节速度 (rad/s): {joint_velocities}")
            print(f"  夹爪位置: {gripper_position:.4f}")
            print()
            
            if i < num_readings - 1:  # 不是最后一次读取
                time.sleep(delay)
                
        except Exception as e:
            print(f"第{i+1}次读取失败: {e}")
            return None
    
    return joint_readings


def analyze_joint_data(joint_readings):
    """
    分析关节数据，检查稳定性和一致性
    
    Args:
        joint_readings (list): 关节角度读取结果列表
    
    Returns:
        dict: 分析结果
    """
    if not joint_readings:
        return None
    
    print("=== 关节数据稳定性分析 ===")
    
    # 提取所有位置数据
    all_positions = np.array([reading['positions'] for reading in joint_readings])
    
    # 计算统计信息
    mean_positions = np.mean(all_positions, axis=0)
    std_positions = np.std(all_positions, axis=0)
    max_positions = np.max(all_positions, axis=0)
    min_positions = np.min(all_positions, axis=0)
    
    print(f"平均关节位置 (rad): {mean_positions}")
    print(f"平均关节位置 (deg): {np.degrees(mean_positions)}")
    print(f"位置标准差 (rad): {std_positions}")
    print(f"位置标准差 (deg): {np.degrees(std_positions)}")
    print(f"最大位置 (rad): {max_positions}")
    print(f"最小位置 (rad): {min_positions}")
    
    # 检查稳定性
    stability_threshold = 0.01  # 1cm的阈值
    is_stable = np.all(std_positions < stability_threshold)
    
    print(f"\n稳定性检查 (阈值: {stability_threshold} rad):")
    for i, std in enumerate(std_positions):
        status = "稳定" if std < stability_threshold else "不稳定"
        print(f"  关节{i+1}: {status} (标准差: {std:.6f} rad)")
    
    print(f"\n整体稳定性: {'稳定' if is_stable else '不稳定'}")
    
    return {
        'mean_positions': mean_positions,
        'std_positions': std_positions,
        'is_stable': is_stable,
        'stability_threshold': stability_threshold
    }


def save_joint_data(joint_readings, analysis_result, filename="arx_joint_data.txt"):
    """
    保存关节数据到文件
    
    Args:
        joint_readings (list): 关节角度读取结果
        analysis_result (dict): 分析结果
        filename (str): 保存文件名
    """
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("ARX机械臂关节角度数据\n")
            f.write("=" * 50 + "\n\n")
            
            f.write("原始读取数据:\n")
            for i, reading in enumerate(joint_readings):
                f.write(f"第{i+1}次读取:\n")
                f.write(f"  时间戳: {reading['timestamp']}\n")
                f.write(f"  关节位置 (rad): {reading['positions']}\n")
                f.write(f"  关节位置 (deg): {np.degrees(reading['positions'])}\n")
                f.write(f"  关节速度 (rad/s): {reading['velocities']}\n")
                f.write(f"  夹爪位置: {reading['gripper_pos']:.4f}\n\n")
            
            if analysis_result:
                f.write("数据分析结果:\n")
                f.write(f"平均关节位置 (rad): {analysis_result['mean_positions']}\n")
                f.write(f"平均关节位置 (deg): {np.degrees(analysis_result['mean_positions'])}\n")
                f.write(f"位置标准差 (rad): {analysis_result['std_positions']}\n")
                f.write(f"整体稳定性: {'稳定' if analysis_result['is_stable'] else '不稳定'}\n")
        
        print(f"数据已保存到文件: {filename}")
        
    except Exception as e:
        print(f"保存数据失败: {e}")


def main():
    """主函数"""
    print("ARX机械臂关节角度读取工具")
    print("=" * 40)
    
    # 配置参数
    model = "X5"  # 可以根据实际情况修改
    interface = "can1"  # 可以根据实际情况修改
    
    # 初始化控制器
    controller = init_arx5_controller(model, interface)
    if controller is None:
        print("初始化失败，程序退出")
        return
    
    try:
        # 等待机械臂稳定
        print("等待机械臂稳定...")
        time.sleep(2.0)
        
        # 读取关节角度
        joint_readings = read_joint_angles(controller, num_readings=5, delay=1.0)
        
        if joint_readings:
            # 分析数据
            analysis_result = analyze_joint_data(joint_readings)
            
            # 保存数据
            save_joint_data(joint_readings, analysis_result)
            
            # 输出最终结果
            print("\n=== 最终结果 ===")
            if analysis_result:
                print(f"起始关节角度 (rad): {analysis_result['mean_positions']}")
                print(f"起始关节角度 (deg): {np.degrees(analysis_result['mean_positions'])}")
                print(f"数据稳定性: {'稳定' if analysis_result['is_stable'] else '不稳定'}")
        
        # 重置到初始位置（可选）
        print("\n是否重置机械臂到初始位置? (y/n): ", end="")
        user_input = input().strip().lower()
        if user_input == 'y':
            print("正在重置到初始位置...")
            controller.reset_to_home()
            print("重置完成")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        print("程序结束")


if __name__ == "__main__":
    main()
