from __future__ import annotations
import socket, threading, time, math, struct, numpy as np
import os
import re
import json
from time import sleep
from typing import List, Sequence, Tuple, Optional
from scipy.spatial.transform import Rotation as R

alarmControllerFile = "files/alarmController.json"
alarmServoFile = "files/alarmServo.json"

_FEEDBACK_DTYPE = np.dtype([('len', np.uint16,),
                   ('reserve', np.byte, (6, )),
                   ('DigitalInputs', np.uint64,),
                   ('DigitalOutputs', np.uint64,),
                   ('RobotMode', np.uint64,),
                   ('TimeStamp', np.uint64,),
                   ('RunTime', np.uint64,),
                   ('TestValue', np.uint64,),
                   ('reserve2', np.byte, (8, )),
                   ('SpeedScaling', np.float64,),
                   ('reserve3', np.byte, (16, )),
                   ('VRobot', np.float64, ),      
                   ('IRobot', np.float64,),
                   ('ProgramState', np.float64,),
                   ('SafetyOIn', np.uint16,),
                   ('SafetyOOut', np.uint16,),
                   ('reserve4', np.byte, (76, )),
                   ('QTarget', np.float64, (6, )),
                   ('QDTarget', np.float64, (6, )),
                   ('QDDTarget', np.float64, (6, )),
                   ('ITarget', np.float64, (6, )),
                   ('MTarget', np.float64, (6, )),
                   ('QActual', np.float64, (6, )),
                   ('QDActual', np.float64, (6, )),
                   ('IActual', np.float64, (6, )),
                   ('ActualTCPForce', np.float64, (6, )),
                   ('ToolVectorActual', np.float64, (6, )),
                   ('TCPSpeedActual', np.float64, (6, )),
                   ('TCPForce', np.float64, (6, )),
                   ('ToolVectorTarget', np.float64, (6, )),
                   ('TCPSpeedTarget', np.float64, (6, )),
                   ('MotorTemperatures', np.float64, (6, )),
                   ('JointModes', np.float64, (6, )),
                   ('VActual', np.float64, (6, )),
                   ('HandType', np.byte, (4, )),
                   ('User', np.byte,),
                   ('Tool', np.byte,),
                   ('RunQueuedCmd', np.byte,),
                   ('PauseCmdFlag', np.byte,),
                   ('VelocityRatio', np.byte,),
                   ('AccelerationRatio', np.byte,),
                   ('reserve5', np.byte, ),
                   ('XYZVelocityRatio', np.byte,),
                   ('RVelocityRatio', np.byte,),
                   ('XYZAccelerationRatio', np.byte,),
                   ('RAccelerationRatio', np.byte,),
                   ('reserve6', np.byte,(2,)),
                   ('BrakeStatus', np.byte,),
                   ('EnableStatus', np.byte,),
                   ('DragStatus', np.byte,),
                   ('RunningStatus', np.byte,),
                   ('ErrorStatus', np.byte,),
                   ('JogStatusCR', np.byte,),   
                   ('CRRobotType', np.byte,),
                   ('DragButtonSignal', np.byte,),
                   ('EnableButtonSignal', np.byte,),
                   ('RecordButtonSignal', np.byte,),
                   ('ReappearButtonSignal', np.byte,),
                   ('JawButtonSignal', np.byte,),
                   ('SixForceOnline', np.byte,),
                   ('CollisionState', np.byte,),
                   ('ArmApproachState', np.byte,),
                   ('J4ApproachState', np.byte,),
                   ('J5ApproachState', np.byte,),
                   ('J6ApproachState', np.byte,),
                   ('reserve7', np.byte, (61, )),
                   ('VibrationDisZ', np.float64,),
                   ('CurrentCommandId', np.uint64,),
                   ('MActual', np.float64, (6, )),
                   ('Load', np.float64,),
                   ('CenterX', np.float64,),
                   ('CenterY', np.float64,),
                   ('CenterZ', np.float64,),
                   ('UserValue[6]', np.float64, (6, )),
                   ('ToolValue[6]', np.float64, (6, )),
                   ('reserve8', np.byte, (8, )),
                   ('SixForceValue', np.float64, (6, )),
                   ('TargetQuaternion', np.float64, (4, )),
                   ('ActualQuaternion', np.float64, (4, )),
                   ('AutoManualMode', np.uint16, ),
                   ('ExportStatus', np.uint16, ),
                   ('SafetyState', np.byte, ),
                   ('reserve9', np.byte,(19,))
                   ])

# Read controller and servo alarm files

def alarmAlarmJsonFile():
    currrntDirectory = os.path.dirname(__file__)
    jsonContrellorPath = os.path.join(currrntDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currrntDirectory, alarmServoFile)

    with open(jsonContrellorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo


class Bestman_Real_CR5:
    """
    • sendRecvMsg(cmd) : 配置 / 查询（阻塞等待返回）
    • _send_raw(cmd)   : 高频运动流 (不等待回包)
    • 实时反馈通过 self.latest_state 提供 (numpy 结构体)
    """

    def __init__(self, ip: str,
                 dash_port: int = 29999,
                 feed_port: int = 30004,
                 text_log: bool = False):
        self._dash = self._open_sock(ip, dash_port, 4096)
        self._feed = self._open_sock(ip, feed_port, 144000)
        self._lock = threading.Lock()          # 保护 dash socket
        self.text_log = text_log
        
        # 实时反馈缓存
        self.latest_state = None
        self._fb_running = True
        threading.Thread(target=self._feedback_loop,
                         daemon=True).start()

        # 上电 & 使能
        self.sendRecvMsg("PowerOn()")
        self.sendRecvMsg("EnableRobot()")

    def log(self, text):
        if self.text_log:
            print(text)
            
    def close(self):
        self._fb_running = False
        for s in (self._dash, self._feed):
            try: s.close()
            except OSError: pass

    # ------- socket 工具 --------------------------------------------------
    @staticmethod
    def _open_sock(ip, port, rcv_buf):
        s = socket.socket();  s.connect((ip, port))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcv_buf)
        return s

    # 发送 + 等待 + 解析（阻塞）
    def sendRecvMsg(self, cmd: str, timeout: float = 3.0) -> str:
        if not cmd.endswith('\n'):
            cmd += '\n'
        with self._lock:
            self._dash.send(cmd.encode())
            self._dash.settimeout(timeout)
            reply = self._dash.recv(1024).decode(errors='ignore')
        self._parse_error(reply, cmd)
        return reply

    # 仅发送（非阻塞，用于 ServoJ 等高频流）
    def _send_raw(self, cmd: str):
        if not cmd.endswith('\n'):
            cmd += '\n'
        with self._lock:
            self._dash.send(cmd.encode())

    # ------ 错误解析（简单打印，可按需 raise） ----------------------------
    @staticmethod
    def _parse_error(reply: str, src_cmd: str):
        # 返回格式 "ErrorID,{…},xxx();"  → 提取首个整数
        try:
            err = int(reply.split(',')[0])
            if err != 0:
                print(f"[CR5‑ERR] {src_cmd.strip()}  ->  {reply.strip()}")
        except ValueError:
            pass   # 非标准返回时忽略

    # ------ 实时反馈线程 --------------------------------------------------
    def _feedback_loop(self):
        """后台常驻读取 1440 B 报文，每次解析常用字段"""
        while self._fb_running:
            try:
                buf = self._feed.recv(1440, socket.MSG_WAITALL)
                if len(buf) >= _FEEDBACK_DTYPE.itemsize:
                    self.latest_state = np.frombuffer(
                        buf[:_FEEDBACK_DTYPE.itemsize],
                        _FEEDBACK_DTYPE)[0]
            except (socket.timeout, ConnectionError):
                pass

    # ------ 基本公共接口 --------------------------------------------------
    def enable_robot(self, load=0.0, centerX=0.0, centerY=0.0, centerZ=0.0, isCheck=-1,):
        """
            可选参数
            参数名 类型 说明
            load double
            设置负载重量，取值范围不能超过各个型号机器⼈的负载范围。单位：kg
            centerX double X⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            centerY double Y⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            centerZ double Z⽅向偏⼼距离。取值范围：-999~ 999，单位：mm
            isCheck int    是否检查负载。1表⽰检查，0表⽰不检查。如果设置为1，则机械臂
            使能后会检查实际负载是否和设置负载⼀致，如果不⼀致会⾃动下使
            能。默认值为0
            可携带的参数数量如下：
            0：不携带参数，表⽰使能时不设置负载重量和偏⼼参数。
            1：携带⼀个参数，该参数表⽰负载重量。
            4：携带四个参数，分别表⽰负载重量和偏⼼参数。
            5：携带五个参数，分别表⽰负载重量、偏⼼参数和是否检查负载。
                """
        string = 'EnableRobot('
        if load != 0:
            string = string + "{:f}".format(load)
            if centerX != 0 or centerY != 0 or centerZ != 0:
                string = string + ",{:f},{:f},{:f}".format(
                    centerX, centerY, centerZ)
                if isCheck != -1:
                    string = string + ",{:d}".format(isCheck)
        string = string + ')'
        return self.sendRecvMsg(string)

    def disable_robot(self):
        """
        Disabled the robot
        下使能机械臂
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def clear_fault(self):      
        self.sendRecvMsg("ClearError()")

    def power_on(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        return self.sendRecvMsg(string)
    
    def stop(self):
        """
       停⽌已下发的运动指令队列或者RunScript指令运⾏的⼯程。
       Stop the delivered motion command queue or the RunScript command from running.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)

    def pause(self):
        """
       暂停已下发的运动指令队列或者RunScript指令运⾏的⼯程。
       Pause the delivered motion command queue or the RunScript command from running.
        """
        string = "Pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
       继续已暂停的运动指令队列或者RunScript指令运⾏的⼯程。
       Continue the paused motion command queue or the RunScript command from running.
        """
        string = "Continue()"
        return self.sendRecvMsg(string)

    def Stop(self):
        """
       停⽌已下发的运动指令队列或者RunScript指令运⾏的⼯程。
       Stop the delivered motion command queue or the RunScript command from running.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)
    
    def brake_control(self, axisID, value):
        """
        描述
        控制指定关节的抱闸。机械臂静⽌时关节会⾃动抱闸，如果⽤⼾需进⾏关节拖拽操作，可开启抱
        闸，即在机械臂下使能状态，⼿动扶住关节后，下发开启抱闸的指令。
        仅能在机器⼈下使能时控制关节抱闸，否则ErrorID会返回-1。
        必选参数
        参数名  类型  说明
        axisID int 关节轴序号，1表⽰J1轴，2表⽰J2轴，以此类推
        value int 设置抱闸状态。0表⽰抱闸锁死（关节不可移动），1表⽰松开抱闸（关节
        可移动）
        Description
        Control the brake of specified joint. The joints automatically brake when the robot is stationary. If you need to drag the joints, you can switch on the brake,
        i.e. hold the joint manually in the disabled status and deliver the command to switch on the brake.
        Joint brake can be controlled only when the robot arm is disabled, otherwise, Error ID will return -1.
        Required parameter:
        Parameter name     Type     Description
        axisID     int     joint ID, 1: J1, 2: J2, and so on
        Value     int     Set the status of brake. 0: switch off brake (joints cannot be dragged). 1: switch on brake (joints can be dragged).
        """
        string = "BrakeControl({:d},{:d})".format(axisID, value)
        return self.sendRecvMsg(string)
    

    
    # ------ 全局速度设置 --------------------------------------------------
    def SpeedFactor(self, speed):
        """
        设置全局速度⽐例。
           机械臂点动时实际运动加速度/速度⽐例 = 控制软件点动设置中的值 x 全局速度⽐例。
           例：控制软件设置的关节速度为12°/s，全局速率为50%，则实际点动速度为12°/s x 50% =
           6°/s
           机械臂再现时实际运动加速度/速度⽐例 = 运动指令可选参数设置的⽐例 x 控制软件再现设置
           中的值 x 全局速度⽐例。
           例：控制软件设置的坐标系速度为2000mm/s，全局速率为50%，运动指令设置的速率为
           80%，则实际运动速度为2000mm/s x 50% x 80% = 800mm/s
        未设置时沿⽤进⼊TCP/IP控制模式前控制软件设置的值。
        取值范围：[1, 100]
        Set the global speed ratio.
           Actual robot acceleration/speed ratio in jogging = value in Jog settings × global speed ratio.
           Example: If the joint speed set in the software is 12°/s and the global speed ratio is 50%, then the actual jog speed is 12°/s x 50% =
           6°/s
           Actual robot acceleration/speed ratio in playback = ratio set in motion command × value in Playback settings
            × global speed ratio.
           Example: If the coordinate system speed set in the software is 2000mm/s, the global speed ratio is 50%, and the speed set in the motion command is
           80%, then the actual speed is 2000mm/s x 50% x 80% = 800mm/s.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Range: [1, 100].
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        设置关节运动⽅式的加速度⽐例。
        未设置时默认值为100
        Set acceleration ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        设置直线和弧线运动⽅式的加速度⽐例。
        未设置时默认值为100。
        Set acceleration ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelJ(self, speed):
        """
        设置关节运动⽅式的速度⽐例。
        未设置时默认值为100。
        Set the speed ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "VelJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelL(self, speed):
        """
        设置直线和弧线运动⽅式的速度⽐例。
        未设置时默认值为100。
        Set the speed ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "VelL({:d})".format(speed)
        return self.sendRecvMsg(string)

    # --- 姿态↔︎四元数 -------------------------------------------------------------------------
    @staticmethod
    def pose_to_euler(p):
        x,y,z,qw,qx,qy,qz = p
        r = R.from_quat([qx,qy,qz,qw])
        roll,pitch,yaw = r.as_euler('xyz', False)
        return [x,y,z, roll,pitch,yaw]

    @staticmethod
    def euler_to_pose(p):
        x,y,z,roll,pitch,yaw = p
        qx,qy,qz,qw = R.from_euler('xyz',[roll,pitch,yaw]).as_quat()
        return [x,y,z,qw,qx,qy,qz]

    # ------ 状态读取 ------------------------------------------------------
    def get_current_robot_mode(self):
        """
        获取机器⼈当前状态。
        1 ROBOT_MODE_INIT 初始化状态
        2 ROBOT_MODE_BRAKE_OPEN 有任意关节的抱闸松开
        3 ROBOT_MODE_POWEROFF 机械臂下电状态
        4 ROBOT_MODE_DISABLED 未使能（⽆抱闸松开）
        5 ROBOT_MODE_ENABLE 使能且空闲
        6 ROBOT_MODE_BACKDRIVE 拖拽模式
        7 ROBOT_MODE_RUNNING 运⾏状态(⼯程，TCP队列运动等)
        8 ROBOT_MODE_SINGLE_MOVE 单次运动状态（点动、RunTo等）
        9 ROBOT_MODE_ERROR
             有未清除的报警。此状态优先级最⾼，⽆论机械臂
             处于什么状态，有报警时都返回9
        10 ROBOT_MODE_PAUSE ⼯程状态
        11 ROBOT_MODE_COLLISION 碰撞检测触发状态
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def get_current_states(self):
        if self.latest_state is None:
            raise RuntimeError("Feedback not yet received")
        # return self.latest_state

        # 将 numpy structured array 转换为字典
        states_dict = {}
        for field_name in self.latest_state.dtype.names:
            value = self.latest_state[field_name]
            if hasattr(value, '__len__') and len(value) > 1:
                # 对于数组类型的字段，转换为列表
                states_dict[field_name] = value.tolist()
            else:
                # 对于标量类型的字段，直接赋值
                states_dict[field_name] = value

        return states_dict

    def get_current_joint_angles(self) -> List[float]:
        st = self.get_current_states()
        return st['QActual']

    def get_current_joint_velocities(self) -> List[float]:
        st = self.get_current_states()
        return st['QDActual']

    def get_current_tcp_speed(self) -> List[float]:
        #st = self.get_current_states()
        #spd = st['TCPSpeedActual']
        #return spd[:3].tolist() + [math.radians(x) for x in spd[3:]]
        #以上返回的是轴角速度
        # 返回row pitch yaw 近似角速度
        st  = self.get_current_states()
        vtr = st['TCPSpeedActual'][:3]           # mm/s
        wvec_deg = st['TCPSpeedActual'][3:] 
        rot = R.from_rotvec(np.radians(wvec_deg))
        roll_deg, pitch_deg, yaw_deg = rot.as_euler('xyz', degrees=True)
        return vtr + [roll_deg, pitch_deg, yaw_deg]
        
    def get_current_end_effector_pose(self) -> List[float]:
        '''
        TCP 位姿  
        [x, y, z,  roll, pitch, yaw]  —— 位置 mm，姿态 度
        将反馈中的 轴角(rx,ry,rz, deg) → 欧拉(roll,pitch,yaw, deg)
        '''
        st  = self.get_current_states()
        x, y, z, rx, ry, rz = st['ToolVectorActual']
        print(x, y, z, rx, ry, rz)
        # 轴角 → Euler
        rot = R.from_rotvec(np.radians([rx, ry, rz]))
        roll_deg, pitch_deg, yaw_deg = rot.as_euler('xyz', degrees=True)

        return [x, y, z, roll_deg, pitch_deg, yaw_deg]

    # ------ 运动指令 (阻塞) -----------------------------------------------
    def _wait_idle(self, poll=0.05):
        while True:
            reply = self.sendRecvMsg("RobotMode()")
            mode = int(re.search(r"\{(\d+)\}", reply).group(1))
            if mode == 5: 
                break
            time.sleep(poll)

    def _build_movl(self, values6,
                use_pose: bool | None = None,   # None / True → pose , False → joint
                user=-1, tool=-1,
                a=-1, v=-1, speed=-1,
                cp=-1, r=-1) -> str:
        """
        MovL 直线插补：默认用 pose=...（coordinateMode=0）
        values6 : pose 或 joint   (mm/deg)
        其它形参 -1 表示不写入指令
        可选参数
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a    int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v    int 执⾏该条指令时的机械臂运动速度⽐例，与speed互斥。取值范围：(0,100]
        speed int 执⾏该条指令时的机械臂运动⽬标速度，与v互斥，若同时存在以speed为
        准。取值范围：[1, 最⼤运动速度]，单位：mm/s
        cp  int 平滑过渡⽐例，与r互斥。取值范围：[0,100]
        r   int 平滑过渡半径，与cp互斥，若同时存在以r为准。单位：mm
        """
        tag = 'pose' if (use_pose is None or use_pose) else 'joint'
        cmd = "MovL(" + tag + "={" + ",".join(f"{x:.3f}" for x in values6) + "}"

        if user  != -1: cmd += f",user={user}"
        if tool  != -1: cmd += f",tool={tool}"
        if a     != -1: cmd += f",a={a}"
        if speed != -1: cmd += f",speed={speed}"
        elif v != -1: cmd += f",v={v}"
        if r != -1: cmd += f",r={r}"
        elif cp != -1: cmd += f",cp={cp}"

        return cmd + ")"
    
    def _build_movj(self, target6,
                use_joint: bool | None = None,
                user=-1, tool=-1,
                a=-1, v=-1, cp=-1) -> str:
        """
        MovJ 关节插补：默认用 joint=...
        values6 : joint 或 pose
        可选参数
        参数名 类型 说明
        user int ⽤⼾坐标系
        tool int ⼯具坐标系
        a int 执⾏该条指令时的机械臂运动加速度⽐例。取值范围：(0,100]
        v int 执⾏该条指令时的机械臂运动速度⽐例。取值范围：(0,100]
        cp int 平滑过渡⽐例。取值范围：[0,100]
        """
        tag = 'joint' if (use_joint is None or use_joint) else 'pose'
        s   = "MovJ(" + tag + "={" + ",".join(f"{x:.3f}" for x in target6) + "}"

        # 2. 可选参数
        if user != -1: s += f",user={user}"
        if tool != -1: s += f",tool={tool}"
        if a    != -1: s += f",a={a}"
        if v    != -1: s += f",v={v}"
        if cp   != -1: s += f",cp={cp}"

        return s + ")"
    
    def move_end_effector_to_goal_pose(self, pose6,
                                   speed: int | None = None,
                                   speed_ratio: int | None = None,
                                   accel_ratio: int | None = None,   
                                   wait: bool = False):
        """
        pose6       : [x, y, z, roll, pitch, yaw]  (mm, deg)
        speed       : 绝对速度 mm/s      —— MovL 参数  speed=...
        speed_ratio : 速度百分比 1‑100       —— MovL 参数  v=...
                    二者都给时以 speed 为准
        accel_ratio : 加速度百分比
        """
        x, y, z, roll, pitch, yaw = pose6
        # RPY(°) → 轴角(°)
        rx, ry, rz = np.degrees(R.from_euler('xyz',
                                         [roll, pitch, yaw],
                                         degrees=True).as_rotvec())      

        opt = {}
        if speed is not None:
            opt['speed'] = int(speed)
        elif speed_ratio is not None:
            opt['v'] = max(1, min(100, int(speed_ratio)))
        if accel_ratio is not None:
            opt['a'] = max(1, min(100, int(accel_ratio)))

        cmd = self._build_movl([x, y, z, rx, ry, rz], **opt)
        self.sendRecvMsg(cmd)
        if wait:
            self._wait_idle()

    def move_arm_to_joint_angles(self, joints_deg,
                             speed_ratio: int = 50,
                             accel_ratio: int | None = None,   
                             wait: bool = False):
        """
        joints_deg  : 长度 6，单位 deg
        speed_ratio : 百分比 1‑100   (v=...)
        """
        v_ratio = max(1, min(100, int(speed_ratio)))
        kwargs = {'v': v_ratio}
        if accel_ratio is not None:
            kwargs['a'] = max(1, min(100, int(accel_ratio)))

        cmd = self._build_movj(joints_deg, **kwargs)   # 默认 joint 方式
        self.sendRecvMsg(cmd)

        if wait:
            self._wait_idle()

    # ------ 高频 ServoJ ---------------------------------------------------
    # ---------- 单步 ServoJ  (Joint space) ---------------------------
    def move_arm_to_joint_angles_servo(self, joints,
                         t=0.01, aheadtime=50.0, gain=500.0):
        """
        joints     : 6×deg
        t          : 本点运行时间 s，默认 = 控制周期
        t float 该点位的运行时间，默认0.1,单位：s 否 [0.004,3600.0]
        aheadtime float 作用类似于PID的D项，默认50，标量，无单位 否 [20.0,100.0]
        gain float 目标位置的比例放大器，作用类似于PID的P项，默认500，标量，无单位 否 [200.0,1000.0]
        """
        cmd = ("ServoJ(" + ",".join(f"{v:.3f}" for v in joints) +
           f",t={t:.4f},aheadtime={aheadtime:.1f},gain={gain:.1f})")
        self._send_raw(cmd)

    def move_arm_to_joint_angles_servo_stream(self, generator, period=0.01):
        """持续迭代发送关节插补点"""
        for j in generator:
            self.move_arm_to_joint_angles_servo(j)
            time.sleep(period)

    # ---------- 单步 ServoP  (TCP pose space) -------------------------
    def move_end_effector_to_goal_pose_servo(self, pose,
                       t=0.01, aheadtime=50.0, gain=500.0):
        """
        pose : [x,y,z,roll,pitch,yaw]  (mm, deg)
        其他参数同 ServoJ
        """
        x, y, z, roll, pitch, yaw = pose
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        rx, ry, rz = np.degrees(rot.as_rotvec())  

        cmd = (f"ServoP({x:.3f},{y:.3f},{z:.3f},"
           f"{rx:.3f},{ry:.3f},{rz:.3f},"
           f"t={t:.4f},aheadtime={aheadtime:.1f},gain={gain:.1f})")
        self._send_raw(cmd)

    def move_end_effector_to_goal_pose_servo_stream(self, generator, period=0.01):
        """持续迭代发送笛卡尔插补点"""
        for p in generator:
            self.move_end_effector_to_goal_pose_servo(p)
            time.sleep(period)

    # ------ IK / FK  -----------------------------------
    def joints_to_cartesian(self, joints_deg: List[float],
                        user: int = -1, tool: int = -1
                       ):
        """
        joints_deg  : 6 × 关节角（单位：度）
        return      : (pos_mm[3], rpy_deg[3])
                    ‑ pos_mm  : XYZ (mm)
                    ‑ rpy_deg : Roll‑Pitch‑Yaw (度，xyz 顺序)
        可选参数
        参数名 类型 说明
        格式为"user=index"，index为已标定的⽤⼾坐标系索引。
        User string 不指定时使⽤全局⽤⼾坐标系。
        Tool string  格式为"tool=index"，index为已标定的⼯具坐标系索引。不指定时使⽤全局⼯具坐标系。
        """
        if len(joints_deg) != 6:
            raise ValueError("joints_deg must have length 6")

        cmd = ("PositiveKin(" + ",".join(f"{v:.3f}" for v in joints_deg) + ")")
        if user != -1:
            cmd = cmd[:-1] + f",user={user})"
        if tool != -1:
            cmd = cmd[:-1] + f",tool={tool})"

        reply = self.sendRecvMsg(cmd)

        # 解析 {x,y,z,rx,ry,rz}
        try:
            payload = reply.split('{')[-1].split('}')[0]
            x, y, z, rx_deg, ry_deg, rz_deg = map(float, payload.split(',')[:6])
        except Exception as e:
            raise RuntimeError(f"PositiveKin parse failed: {reply}") from e

        # 轴角(°) → Roll‑Pitch‑Yaw(°)
        rotvec_rad = [math.radians(a) for a in (rx_deg, ry_deg, rz_deg)]
        rpy_deg = (R.from_rotvec(rotvec_rad)
                    .as_euler('xyz', degrees=True)
                    .tolist())
        Roll, Pitch, Yaw = rpy_deg
        return x, y, z, Roll, Pitch, Yaw

    def cartesian_to_joints(self, pose_deg: List[float],
                        user: int = -1, tool: int = -1,
                        useJointNear: int = -1, JointNear: str = ''
                       ) -> List[float]:
        """
        pose_deg : [x, y, z, roll, pitch, yaw]  (单位：mm, 度)
        return   : 6 × 关节角（度）
        可选参数
        参数名 类型 说明
        User string  格式为"user=index"，index为已标定的⽤⼾坐标系索引。不指定时使⽤全局⽤⼾坐标系。
        Tool string  格式为"tool=index"，index为已标定的⼯具坐标系索引。不指定时使⽤全局⼯具坐标系。
        useJointNear string  ⽤于设置JointNear参数是否有效。
            "useJointNear=0"或不携带表⽰JointNear⽆效，系统根据机械臂当前关节⻆度就近选解。
            "useJointNear=1"表⽰根据JointNear就近选解。
        jointNear string 格式为"jointNear={j1,j2,j3,j4,j5,j6}"，⽤于就近选解的关节坐标。
        """
        if len(pose_deg) != 6:
            raise ValueError("pose_deg must be length 6")

        x, y, z, roll_deg, pitch_deg, yaw_deg = pose_deg

        # RPY(°) → 轴角(°)
        rotvec_rad = R.from_euler('xyz',
                              [math.radians(roll_deg),
                               math.radians(pitch_deg),
                               math.radians(yaw_deg)],
                              degrees=False).as_rotvec()
        rx_deg, ry_deg, rz_deg = [a * 180 / math.pi for a in rotvec_rad]

        cmd = (f"InverseKin({x:.3f},{y:.3f},{z:.3f},"
            f"{rx_deg:.3f},{ry_deg:.3f},{rz_deg:.3f}")

        # 追加可选参数
        if user != -1:         cmd += f",user={user}"
        if tool != -1:         cmd += f",tool={tool}"
        if useJointNear != -1: cmd += f",useJointNear={useJointNear}"
        if JointNear:          cmd += f",JointNear={JointNear}"
        cmd += ")"

        reply = self.sendRecvMsg(cmd)

        # 解析 {j1,j2,j3,j4,j5,j6}
        try:
            payload = reply.split('{')[-1].split('}')[0]
            joints_deg = list(map(float, payload.split(',')[:6]))
        except Exception as e:
            raise RuntimeError(f"InverseKin parse failed: {reply}") from e

        return joints_deg

    # ------ gripper -----------------------------------------
    def SetTool485(self, baud,
               parity: str = "N",          # N / O / E
               stopbit: int = 1,           # 1 / 2
               identify: int = -1) -> str:
        """
        描述:
        设置末端⼯具的RS485接⼝对应的数据格式。
        必选参数
        参数名 类型 说明
        baud int RS485接⼝的波特率
        可选参数
        参数名 类型 说明
        parity string
        是否有奇偶校验位。"O"表⽰奇校验，"E"表⽰偶校验，"N"表⽰⽆奇偶
        校验位。默认值为“N”。
        stopbit int 停⽌位⻓度。取值范围：1，2。默认值为1。
        identify int 当机械臂为多航插机型时，⽤于指定设置的航插。1：航插1；2：航插2
        返回
        ErrorID,{},SetTool485(baud,parity,stopbit);
        ⽰例：
        SetTool485(115200,"N",1)
        将末端⼯具的RS485接⼝对应的波特率设置为115200Hz，⽆奇偶校验位，停⽌位⻓度为1。
        """
        if parity not in {"N", "E", "O"}:
            raise ValueError("parity must be 'N', 'E' or 'O'")
        if stopbit not in {1, 2}:
            raise ValueError("stopbit must be 1 or 2")

        pieces = [f"{baud}", f"\"{parity}\"", f"{stopbit}"]
        if identify != -1:
            pieces.append(str(identify))
        cmd = f"SetTool485({','.join(pieces)})"
        return self.sendRecvMsg(cmd)
    
    def ModbusCreate(self, ip, port, slave_id, isRTU=-1):
            """
            创建Modbus主站，并和从站建⽴连接。最多⽀持同时连接5个设备。
            必选参数
            参数名 类型 说明
            ip string 从站IP地址
            port int 从站端⼝
            slave_id int 从站ID
            可选参数
            参数名 类型 说明
            isRTU int 如果不携带或为0，建⽴modbusTCP通信； 如果为1，建⽴modbusRTU通信
            """
            string = "ModbusCreate({:s},{:d},{:d}".format(ip, port, slave_id)
            params = []
            if isRTU != -1:
                params.append('{:d}'.format(isRTU))
            for ii in params:
                string = string + ',' + ii
            string = string + ')'
            return self.sendRecvMsg(string)

    def ModbusRTUCreate(self, slave_id:int, baud:int,
                        parity:str="N", data_bit:int=8,
                        stop_bit:int=1) -> str:
        """
        创建基于RS485接⼝的Modbus主站，并和从站建⽴连接。最多⽀持同时连接5个设备。
        必选参数
        参数名 类型 说明
        slave_id int 从站ID
        baud int RS485接⼝的波特率。
        可选参数
        参数名 类型 说明
        parity string
        是否有奇偶校验位。"O"表⽰奇校验，"E"表⽰偶校验，"N"表⽰⽆奇偶
        校验位。默认值为“E”。
        data_bit int 数据位⻓度。取值范围：8。默认值为8。
        stop_bit int 停⽌位⻓度。取值范围：1，2。默认值为1。
        """
        cmd = f"ModbusRTUCreate({slave_id},{baud},\"{parity}\",{data_bit},{stop_bit})"
        return self.sendRecvMsg(cmd)
    
    def ModbusClose(self, index):
        """
        和Modbus从站断开连接，释放主站。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        Disconnect with Modbus slave and release the master.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        """
        string = "ModbusClose({:d})".format(index)
        return self.sendRecvMsg(string)
    
    def SetHoldRegs(self, idx:int, addr:int, n:int, tab:str, tp:str='U16'):
        """
        将指定的值以指定的数据类型写⼊Modbus从站保持寄存器指定的地址。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 保持寄存器起始地址
        count int 连续写⼊保持寄存器的值的数量。取值范围：[1, 4]
        valTab string 要写⼊的值，数量与count相同。
        可选参数
        参数名 类型 说明
        valType string
        写⼊的数据类型：
        U16：16位⽆符号整数（2个字节，占⽤1个寄存器）；
        U32：32位⽆符号整数（4个字节，占⽤2个寄存器）
        F32：32位单精度浮点数（4个字节，占⽤2个寄存器）
        F64：64位双精度浮点数（8个字节，占⽤4个寄存器）
        默认为U16
        """
        return self.sendRecvMsg(f"SetHoldRegs({idx},{addr},{n},{tab},{tp})")
    
    def GetHoldRegs(self, idx:int, addr:int, n:int, tp:str='U16'):
        """
        按照指定的数据类型，读取Modbus从站保持寄存器地址的值。
        必选参数
        参数名 类型 说明
        index int 创建主站时返回的主站索引
        addr int 保持寄存器起始地址
        count int 连续读取保持寄存器的值的数量。取值范围：[1, 4]
        可选参数
        参数名 类型 说明
        valType string
        读取的数据类型：
        U16：16位⽆符号整数（2个字节，占⽤1个寄存器）；
        U32：32位⽆符号整数（4个字节，占⽤2个寄存器）
        F32：32位单精度浮点数（4个字节，占⽤2个寄存器）
        F64：64位双精度浮点数（8个字节，占⽤4个寄存器）
        默认为U16
        """
        return self.sendRecvMsg(f"GetHoldRegs({idx},{addr},{n},{tp})")
                
    def _ensure_gripper_modbus(self,
                            slave_id:int = 3,
                            baud:int = 115200,
                            parity:str = "N",
                            stopbit:int = 1):
        """
        1. 若已有主站索引且 >0 则直接返回。
        2. 否则   → SetTool485 → ModbusRTUCreate  (串口 RS-485)
        3. 解析返回串 '0,{idx}'，idx 必须 >0。
        4. 完成复位 + 激活。
        """
        if getattr(self, "_mb_idx", 0) > 0:
            return                                     # 已 OK

        # ---------- ① 配好 485 ----------
        self.SetTool485(baud, parity, stopbit)

        # ---------- ② 创建 RTU 主站 ----------
        rep = self.ModbusRTUCreate(slave_id, baud, parity, 8, stopbit)
        if not rep.startswith("0,"):
            raise RuntimeError(f"ModbusRTUCreate 失败: {rep}")

        m = re.search(r'\{\s*(\d+)\s*\}', rep)
        if not m or int(m.group(1)) == 0:
            raise RuntimeError(f"未获取到有效主站索引: {rep}")
        self._mb_idx = int(m.group(1))                # ≥1

        # ---------- ③ 复位 + 激活 ----------
        self._wr_u16(0x03E8, 0x0000)                  # reset
        time.sleep(0.1)
        self._wr_u16(0x03E8, 0x0900)                  # rACT=1, rGTO=1

        t0 = time.time()
        while time.time() - t0 < 5:
            sta_hi = self._rd_u16(0x07D0, 1)[0] >> 8  # gSTA 高字节
            if sta_hi & 0b00011000 == 0b00011000:     # gACT=1 & gGTO=1
                return                                # 激活成功
            time.sleep(0.05)

        raise TimeoutError("Robotiq 夹爪激活超时 (>5 s)")

    def _wr_u16(self, addr:int, *vals:int):
        tab = '{' + ','.join(str(v & 0xFFFF) for v in vals) + '}'
        self.SetHoldRegs(self._mb_idx, addr, len(vals), tab, 'U16')

    def _rd_u16(self, addr:int, n:int=1) -> list[int]:
        rep  = self.GetHoldRegs(self._mb_idx, addr, n, 'U16')
        nums = list(map(int, re.findall(r'\d+', rep.split('{',1)[1])))
        return nums[:n]

    def _send_gripper_command(self, pos:int,
                            spd:int=0xFF, frc:int=0xFF,
                            rACT:int=1, rGTO:int=1,
                            rATR:int=0, rMOD:int=0):
        """3 × u16 写到 0x03E8~0x03EA"""
        self._ensure_gripper_modbus()

        ctrl = (rACT << 8) | (rGTO << 11) | (rATR << 12) | (rMOD << 13)
        self._wr_u16(0x03E8, ctrl)
        self._wr_u16(0x03E9, pos & 0xFF)
        self._wr_u16(0x03EA, ((spd & 0xFF) << 8) | (frc & 0xFF))

    def gripper_goto(self, value: int,
                 speed: int = 0xFF,   # 0‑255
                 force: int = 0xFF,   # 0‑255
                 wait: bool = False,
                 timeout: float = 2.0):
        """
        把夹爪夹到 value (0=全开, 255=全闭)，速度 / 力可选。
        """
        self._send_gripper_command(value, speed, force)

        if wait:
            t0 = time.time()
            while time.time() - t0 < timeout:
                if abs(self.get_gripper_position() - value) <= 3:
                    break
                time.sleep(0.05)
    
    def open_gripper(self, wait: bool = True):
        self.gripper_goto(0, wait=wait)

    def close_gripper(self, wait: bool = True):  
        self.gripper_goto(255, wait=wait)
    
    def get_gripper_position(self) -> int:
        """返回 gPO (0-255)，0=张开，255=闭合"""
        self._ensure_gripper_modbus()
        return self._rd_u16(0x07D8, 1)[0] & 0xFF   # 0x07D0 + 8

    # ------ 力控指令 -----------------------------------------
    def EnableFTSensor(self, status):
        """
        开启/关闭力传感器。
        """
        string = "EnableFTSensor({:d})".format(status)
        return self.sendRecvMsg(string)

    def SixForceHome(self):
        """
        将力传感器当前数值置0，即以传感器当前受力状态作为零点。
        """
        string = "SixForceHome()"
        return self.sendRecvMsg(string)
    
    def GetForce(self, tool = -1):
        """
        获取力传感器当前数值。
        tool int 用于指定获取数值时参考的工具坐标系，取值范围：[0,50]。
        不指定时使用全局工具坐标系
        """
        if tool == -1:
            string = "GetForce()"
        else:
            string = "GetForce({:d})".format(tool)
        return self.sendRecvMsg(string)