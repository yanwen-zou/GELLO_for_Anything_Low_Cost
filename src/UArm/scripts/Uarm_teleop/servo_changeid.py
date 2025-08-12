import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'  # 替换为你的串口号
BAUD_RATE = 115200

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
    # 修改 ID：将原 ID=000 改为 000
    cmd = b'#000PID002!\r\n'
    ser.write(cmd)
    print(f"已发送修改ID命令：{cmd.decode().strip()}")
    time.sleep(0.5)

    if ser.in_waiting:
        response = ser.read(ser.in_waiting).decode(errors='ignore')
        print(f"返回：{response}")
    else:
        print("未收到舵机响应（可能修改成功但未回传）")
