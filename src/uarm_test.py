import uarm
from uarm.tools.list_ports import get_ports

# uv pip install "git+https://github.com/uArm-Developer/uArm-Python-SDK.git#egg=uarm"
# 终端里输入命令可以得到端口信息 ls /dev/tty.*
# 自动连接到你的端口 /dev/tty.usbmodem1201
arm = uarm.SwiftAPI(port='/dev/tty.usbmodem1201')
arm.connect()

if arm.connected:
    print("连接成功！固件版本:", arm.get_device_info())
    # 移动到安全高度
    # arm.set_position(x=200, y=0, z=150, speed=50, wait=True)
    arm.reset()
    print(arm.get_servo_angle())
    # print(arm.send_cmd_sync("G28"))

    # 关掉电机（可以手动拖动）
    # arm.set_servo_detach()
    # print("电机已释放，你可以手动摆动机械臂了。")
else:
    print("Python 仍然无法握手，请检查端口占用。")
arm.disconnect()
