from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.199')
arm.connect()
print(arm.get_state())
