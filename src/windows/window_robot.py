# -*- coding: utf-8 -*-
"""
Created on Tues Aug  3 17:06:02 2021

@author: wmy and wjx
"""
import serial
import serial.tools.list_ports
import threading
import tkinter as tk
from tkinter import ttk
import re
import queue

import time
import socket
import numpy as np
from math import sqrt

from robot.tools import YamlHandler, FunctionFitter, TrilinearCalibrator
from robot.robot_ik import inverse_kinematics
from robot import SERVER_ADDR, ROOT, LOG

ROBOT_PARAMS = ROOT + "/robot_params.yaml"

PER_ANGLE_TIME = 15  # 机械臂运行速度：舵机转动一度需要的时间，ms
INIT_ENGINE = 120
ENGINE_NUM = 3

TOOL_LENGTH = 69  # 工具长度，洗泵延长距离为 69mm
INIT_ENGINE_Y = INIT_ENGINE + TOOL_LENGTH  # 初始引擎Y轴位置

# for uarm
MANUAL_COMPENSATE_X = 0
MANUAL_COMPENSATE_Y = 0
MANUAL_COMPENSATE_Z = 0

POINT_NUM = 9  # 设置校准点位数量
NINE_POINT = [
    [150, -85, 19],
    [150, 0, 19],
    [150, 85, 19],
    [236, 85, 19],
    [236, 0, 19],
    [236, -85, 19],
    [323, -85, 19],
    [323, 0, 19],
    [323, 85, 19],
]
TEST_NINE_POINT = NINE_POINT

# for openarm
# MANUAL_COMPENSATE_X = -1
# MANUAL_COMPENSATE_Y = 0
# MANUAL_COMPENSATE_Z = -5

# POINT_NUM = 18  # 设置校准点位数量
# NINE_POINT = [
#     [-100, 149, 27],  # 台面 96mm
#     [0, 149, 27],
#     [100, 149, 27],
#     [100, 199, 27],
#     [0, 199, 27],
#     [-100, 199, 27],
#     [-100, 224, 27],
#     [0, 224, 27],
#     [100, 224, 27],
#     [-100, 149, -70],  # 桌面，底座平齐
#     [0, 149, -70],
#     [100, 149, -70],
#     [100, 199, -70],
#     [0, 199, -70],
#     [-100, 199, -70],
#     [-100, 224, -70],
#     [0, 224, -70],
#     [100, 224, -70],
# ]

# TEST_NINE_POINT = [
#     # [-100, 149, -70],
#     # [0, 149, -70],
#     # [100, 149, -70],
#     # [100, 199, -70],
#     # [0, 199, -70],
#     # [-100, 199, -70],
#     # [-100, 224, -70],
#     # [0, 224, -70],
#     # [100, 224, -70],
#     [-100, 149, -50],  # 原点高度130-棋盘和棋子高度19-吸嘴长度61
#     [0, 149, -50],
#     [100, 149, -50],
#     [100, 199, -50],
#     [0, 199, -50],
#     [-100, 199, -50],
#     [-100, 224, -50],
#     [0, 224, -50],
#     [100, 224, -50],
# ]


class RobotSerialPortWindow:
    def __init__(self, window_flag_bit=None):
        self.serial = serial.Serial()
        self.arm = "uarm"
        self.device = None
        self.baudrate = 115200
        self.encoding = "utf-8"
        self.recthread = None
        self.connecting = False
        self.comports = []
        self.devices = []
        self.search()
        self.thread_open_flag = True
        self.updatethread = threading.Thread(target=self.update)
        self.updatethread.setDaemon(True)
        self.updatethread.start()

        self.working__flag = True
        self.load_fit()

        self.command_completed_event = threading.Event()

        self.last_angle_list = [0.0, 0.0, 0.0]
        self.last_engine_list = [0, INIT_ENGINE_Y, INIT_ENGINE]
        self.per_angle_time = PER_ANGLE_TIME  # 舵机转动一度需要的时间，ms

        self.window_flag_bit = window_flag_bit
        self.window()

    def window(self):
        self.root = tk.Tk()
        self.root.title("机械臂控制器(上位机)")
        self.root.geometry("960x640")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.face = tk.Frame(self.root)
        self.face.config(height=640, width=725, bd=1)
        self.face.propagate(False)
        self.face.pack(anchor="nw", side="left")

        textframe1 = tk.Frame(
            self.face,
            highlightbackground="red",
            highlightcolor="red",
            highlightthickness=2,
            bd=0,
        )
        textframe1.config(height=140, width=725, bd=1, relief="solid", bg="#a0a0a0")
        textframe1.propagate(False)
        textframe1.pack(anchor="nw", side="bottom")

        # display client command text
        self.rectext1 = tk.Text(
            textframe1, height=35, width=99, bg="black", fg="#00FF00"
        )
        self.rectext1.insert(
            tk.END,
            "接收客户端指令如下：\n",
        )
        self.recscrollbar1 = tk.Scrollbar(textframe1)
        self.rectext1["yscrollcommand"] = self.recscrollbar1.set
        self.rectext1.config(state=tk.DISABLED)
        self.recscrollbar1["command"] = self.rectext1.yview
        self.rectext1.pack(side=tk.LEFT, fill=tk.BOTH)
        self.recscrollbar1.pack(side="left", fill=tk.Y)

        # operate frame
        operateframe = tk.Frame(
            self.face,
            highlightbackground="green",
            highlightcolor="green",
            highlightthickness=2,
            bd=0,
        )
        operateframe.config(height=140, width=725, relief="solid", bg="#a0a0a0")
        operateframe.propagate(False)
        operateframe.pack(anchor="nw", side="bottom")

        # send text
        operatetextframe = tk.Frame(operateframe)
        operatetextframe.config(height=140, width=725)
        operatetextframe.propagate(False)
        operatetextframe.pack(anchor="nw", side="left")

        operatespaceframe = tk.Frame(operatetextframe)
        operatespaceframe.config(height=10, width=725)
        operatespaceframe.propagate(False)
        operatespaceframe.pack(anchor="sw", side="bottom")

        # operate right
        operateframeright = tk.Frame(operateframe)
        operateframeright.config(height=150, width=225)
        operateframeright.propagate(False)
        operateframeright.pack(anchor="nw", side="left")

        # text
        self.sendtext = tk.Text(
            operatetextframe, height=15, width=99, bg="white", fg="black"
        )
        self.sendtext.insert(
            tk.END,
            "在此输入你想发送的指令 回车结尾。例如: G0 X100 Y100 Z100 F50 回车 ,注:坐标值不会被纠偏",
        )
        self.sendscrollbar = tk.Scrollbar(operatetextframe)
        self.sendtext["yscrollcommand"] = self.sendscrollbar.set
        self.sendscrollbar["command"] = self.sendtext.yview
        self.sendtext.pack(side=tk.LEFT)
        self.sendscrollbar.pack(side="left", fill=tk.Y)

        # text frame
        textframe = tk.Frame(
            self.face,
            highlightbackground="blue",
            highlightcolor="blue",
            highlightthickness=2,
            bd=0,
        )
        textframe.config(height=350, width=725, relief="solid", bg="#a0a0a0")
        textframe.propagate(False)
        textframe.pack(anchor="nw", side="left")

        # serial text black, 展示串口数据
        self.rectext = tk.Text(textframe, height=35, width=99, bg="black", fg="#00FF00")
        self.rectext.insert(
            tk.END,
            "机械臂响应结果如下：\n",
        )
        self.recscrollbar = tk.Scrollbar(textframe)
        self.rectext["yscrollcommand"] = self.recscrollbar.set
        self.rectext.config(state=tk.DISABLED)
        self.recscrollbar["command"] = self.rectext.yview
        self.rectext.pack(side=tk.LEFT, fill=tk.BOTH)
        self.recscrollbar.pack(side="left", fill=tk.Y)

        # right frame
        rightframe = tk.Frame(
            self.root,
            highlightbackground="yellow",
            highlightcolor="yellow",
            highlightthickness=2,
            bd=0,
        )
        rightframe.config(height=640.0, width=225, relief="solid", bg="#a0a0a0")
        rightframe.propagate(False)
        rightframe.pack(anchor="nw", side="left")

        optionframe = tk.Frame(rightframe)
        optionframe.config(height=180.0, width=225, relief="solid", bg="#a0a0a0")
        optionframe.propagate(False)
        optionframe.pack(anchor="nw", side="top")

        # left
        optionframeleft = tk.Frame(optionframe)
        optionframeleft.config(height=180.0, width=60)
        optionframeleft.propagate(False)
        optionframeleft.pack(anchor="nw", side="left")
        # right
        optionframeright = tk.Frame(optionframe)
        optionframeright.config(height=180.0, width=165)
        optionframeright.propagate(False)
        optionframeright.pack(anchor="nw", side="left")
        # bottom
        optionframebottom = tk.Frame(rightframe)
        optionframebottom.config(height=450.0, width=225)
        optionframebottom.propagate(False)
        optionframebottom.pack(anchor="nw", side="bottom")

        # arm type
        label0 = tk.Label(optionframeleft, text="机械臂", width=5, height=1)
        label0.pack(pady=2)
        self.armselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.armselect.bind("<<ComboboxSelected>>", self.armselectcmd)
        self.armselect["value"] = ["uarm", "openarm"]
        self.armselect.current(0)
        self.armselect.pack()

        # serial
        label1 = tk.Label(optionframeleft, text="端口号", width=5, height=1)
        label1.pack(pady=2)
        self.serialselect = ttk.Combobox(optionframeright, width=15, height=5)
        self.serialselect.bind("<<ComboboxSelected>>", self.serialselectcmd)
        self.serialselect.pack()

        # baudrate
        label2 = tk.Label(optionframeleft, text="波特率", width=5, height=1)
        label2.pack(pady=2)
        self.baudrateselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.baudrateselect.bind("<<ComboboxSelected>>", self.baudrateselectcmd)
        self.baudrateselect["value"] = [
            1382400,
            921600,
            460800,
            256000,
            230400,
            128000,
            115200,
            76800,
            57600,
            43000,
            38400,
            19200,
            14400,
            9600,
            4800,
            2400,
            1200,
        ]
        self.baudrateselect.current(6)
        self.baudrateselect.pack()
        # cal bit
        label3 = tk.Label(optionframeleft, text="校验位", width=5, height=1)
        label3.pack(pady=2)
        self.calbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.calbitselect["value"] = ["无校验", "奇校验", "偶校验"]
        self.calbitselect.current(0)
        self.calbitselect.pack()
        # data bit
        label4 = tk.Label(optionframeleft, text="数据位", width=5, height=1)
        label4.pack(pady=2)
        self.databitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.databitselect["value"] = [8, 7, 6, 5]
        self.databitselect.current(0)
        self.databitselect.pack()
        # stop bit
        label5 = tk.Label(optionframeleft, text="停止位", width=5, height=1)
        label5.pack(pady=2)
        self.stopbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.stopbitselect["value"] = [1]
        self.stopbitselect.current(0)
        self.stopbitselect.pack()

        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.openbutton = tk.Button(
            optionframebottom,
            text="连接机械臂",
            width=20,
            height=1,
            command=self.openbuttoncmd,
        )
        self.openbutton.pack()

        # 上位机
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.calcparambutton = tk.Button(
            optionframebottom,
            text="机械臂校准",
            width=20,
            height=1,
            command=self.calcparam,
        )
        self.calcparambutton.pack()

        # send botton
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.sendbutton = tk.Button(
            optionframebottom,
            text="发送指令",
            width=20,
            height=1,
            command=self.sendbuttoncmd,
        )
        self.sendbutton.pack()

        # # suck botton
        # spacelabel = tk.Label(optionframebottom, width=5, height=1)
        # spacelabel.pack()
        # self.suckupbutton = tk.Button(
        #     optionframebottom,
        #     text="打开吸泵",
        #     width=20,
        #     height=1,
        #     command=self.suckup,
        # )
        # self.suckupbutton.pack()

        # # suck down
        # spacelabel = tk.Label(optionframebottom, width=5, height=1)
        # spacelabel.pack()
        # self.suckdownbutton = tk.Button(
        #     optionframebottom,
        #     text="关闭吸泵",
        #     width=20,
        #     height=1,
        #     command=self.suckdown,
        # )
        # self.suckdownbutton.pack()

        # remote
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.locatebutton = tk.Button(
            optionframebottom,
            text="棋盘定位",
            width=20,
            height=1,
            command=self.locatebuttoncmd,
        )
        self.locatebutton.pack()

        # 开启监听下棋指令
        # todo：链接机械臂时自动开启监听
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.runbutton = tk.Button(
            optionframebottom,
            text="开启指令监听",
            width=20,
            height=1,
            command=self.runbuttoncmd,
        )
        self.runbutton.pack()

        self.display_queue = queue.Queue()
        self.process_display_operations()

        self.root.mainloop()

    def process_display_operations(self):
        """在主线程中处理OpenCV操作"""
        try:
            while True:
                operation = self.display_queue.get_nowait()
                operation()
        except queue.Empty:
            pass
        # 安排下一次处理
        self.root.after(10, self.process_display_operations)

    def close(self):
        try:
            self.reset()
            self.server.close()
        except:
            pass
        self.working__flag = False
        self.thread_open_flag = False
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 1)
        if self.serial.isOpen():
            self.serialclose()
        self.root.withdraw()  # 一次运行中多次开关此界面会造成内存泄露，但是使用destroy()会卡死GUI，只能后续再改进了

    def calcparam(self):
        if self.calcparambutton["text"] == "机械臂校准":
            if not self.serial.isOpen():
                tk.messagebox.showerror(
                    title="错误", message="请先连接机械臂", parent=self.root
                )
                return

            self.calcparambutton["text"] = "停止校准"
            self.paramwindow = tk.Toplevel(self.face)
            self.paramwindow.title("机械臂内参校准")
            self.paramwindow.geometry("960x640")
            self.paramwindow.protocol("WM_DELETE_WINDOW", self.close_paramwindow)

            self.point_count = 0

            zerobutton = tk.Button(
                self.paramwindow,
                text="回到初始位置(0,189,120)",
                command=self.zerobuttoncmd,
            )
            zerobutton.grid(row=0, column=1, padx=20, pady=20, sticky=("e", "w"))

            initposbutton = tk.Button(
                self.paramwindow, text="复位/位置查找", command=self.initposbuttoncmd
            )
            initposbutton.grid(row=0, column=0, padx=20, pady=20, sticky=("e", "w"))

            curposbutton = tk.Button(
                self.paramwindow, text="获取当前位置坐标", command=self.curposbuttoncmd
            )
            curposbutton.grid(row=0, column=2, padx=20, pady=20, sticky=("e", "w"))

            suckupbutton = tk.Button(
                self.paramwindow, text="打开吸泵/继电器", command=self.suckup
            )
            suckupbutton.grid(row=0, column=4, padx=20, pady=20, sticky=("e", "w"))
            suckdownbutton = tk.Button(
                self.paramwindow, text="关闭吸泵/继电器", command=self.suckdown
            )
            suckdownbutton.grid(row=0, column=5, padx=20, pady=20, sticky=("e", "w"))

            gripperonbutton = tk.Button(
                self.paramwindow, text="打开夹子/舵机", command=self.gripperon
            )
            gripperonbutton.grid(row=1, column=4, padx=20, pady=20, sticky=("e", "w"))
            gripperoffbutton = tk.Button(
                self.paramwindow, text="关闭夹子/舵机", command=self.gripperoff
            )
            gripperoffbutton.grid(row=1, column=5, padx=20, pady=20, sticky=("e", "w"))

            DConbutton = tk.Button(
                self.paramwindow, text="打开12V直流输出", command=self.dc_level_on
            )
            DConbutton.grid(row=2, column=4, padx=20, pady=20, sticky=("e", "w"))
            DCoffbutton = tk.Button(
                self.paramwindow, text="关闭12V直流输出", command=self.dc_level_off
            )
            DCoffbutton.grid(row=2, column=5, padx=20, pady=20, sticky=("e", "w"))

            steplabel = tk.Label(self.paramwindow, text="步长:")
            steplabel.grid(row=1, column=0, padx=20, pady=20, sticky=("e", "w"))
            stepinp = tk.Entry(self.paramwindow)
            stepinp.insert(0, "5")
            stepinp.grid(row=1, column=1, padx=20, pady=20, sticky=("e", "w"))
            self.stepinp = stepinp

            reducebutton0 = tk.Button(
                self.paramwindow, text="X-", command=self.reducebutton0cmd
            )
            reducebutton0.grid(row=2, column=0, padx=20, pady=20, sticky=("e", "w"))

            addbutton0 = tk.Button(
                self.paramwindow, text="X+", command=self.addbutton0cmd
            )
            addbutton0.grid(row=2, column=2, padx=20, pady=20, sticky=("e", "w"))

            reducebutton1 = tk.Button(
                self.paramwindow, text="Y-", command=self.reducebutton1cmd
            )
            reducebutton1.grid(row=3, column=0, padx=20, pady=20, sticky=("e", "w"))

            addbutton1 = tk.Button(
                self.paramwindow, text="Y+", command=self.addbutton1cmd
            )
            addbutton1.grid(row=3, column=2, padx=20, pady=20, sticky=("e", "w"))

            reducebutton2 = tk.Button(
                self.paramwindow, text="Z-", command=self.reducebutton2cmd
            )
            reducebutton2.grid(row=4, column=0, padx=20, pady=20, sticky=("e", "w"))

            addbutton2 = tk.Button(
                self.paramwindow, text="Z+", command=self.addbutton2cmd
            )
            addbutton2.grid(row=4, column=2, padx=20, pady=20, sticky=("e", "w"))

            targetbutton = tk.Button(
                self.paramwindow, text="开始点位校准", command=self.do_first_point
            )
            targetbutton.grid(row=5, column=0, padx=20, pady=20, sticky=("e", "w"))

            addparabutton = tk.Button(
                self.paramwindow, text="确定该点已校准", command=self.addparabuttoncmd
            )
            addparabutton.grid(row=5, column=1, padx=20, pady=20, sticky=("e", "w"))

            calcbutton = tk.Button(
                self.paramwindow, text="停止校准", command=self.stop_calc
            )
            calcbutton.grid(row=5, column=2, padx=20, pady=20, sticky=("e", "w"))

            testninebutton = tk.Button(
                self.paramwindow,
                text="点位移动测试(校准)",
                command=self.testninebuttoncmd,
            )
            testninebutton.grid(row=6, column=1, padx=20, pady=20, sticky=("e", "w"))
            testninebutton2 = tk.Button(
                self.paramwindow,
                text="点位移动测试(未校准)",
                command=self.testninebuttoncmd2,
            )
            testninebutton2.grid(row=6, column=0, padx=20, pady=20, sticky=("e", "w"))

            self.len_params = []

            self.engine_real = np.zeros((3, POINT_NUM))
            self.engine_model = np.zeros((3, POINT_NUM))

        else:
            self.close_paramwindow()

    def close_paramwindow(self):
        self.stop_calc()
        self.paramwindow.destroy()
        self.calcparambutton["text"] = "机械臂校准"

    def stop_calc(self):
        self.point_count = 0
        self.len_params = []
        self.engine_real = np.zeros((3, POINT_NUM))
        self.engine_model = np.zeros((3, POINT_NUM))

        # self.restoration()

    def testninebuttoncmd(self):
        if self.connecting:
            for i in range(len(TEST_NINE_POINT)):
                offset = TEST_NINE_POINT[i]
                self.move(offset, need_fit=True)
                time.sleep(5)

            # self.stop_calc()
            # self.do_first_point(need_fit=True)
            # time.sleep(2)
            # for i in range(POINT_NUM - 1):
            #     self.do_next_point(need_fit=True)
            #     time.sleep(2)
            self.zerobuttoncmd()
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def testninebuttoncmd2(self):
        if self.connecting:
            for i in range(POINT_NUM):
                offset = NINE_POINT[i]
                self.move(offset, need_fit=False)
                time.sleep(5)
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def do_first_point(self, need_fit=False):
        self.stop_calc()
        if self.connecting:
            LOG.info(f"start do_first_point, need_fit: {need_fit}")
            self.point_count = 0
            offset = NINE_POINT[self.point_count]
            self.move(offset, need_fit=need_fit)
            for i in range(ENGINE_NUM):
                self.engine_model[i][self.point_count] = self.last_engine_list[i]
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def do_next_point(self, need_fit=False):
        if self.connecting:
            self.point_count = self.point_count + 1
            # offset = NINE_POINT[self.point_count - 1]
            # self.robotrun([offset[0], offset[1], offset[2] + 30])
            offset = NINE_POINT[self.point_count]
            # self.robotrun([offset[0], offset[1], offset[2] + 30])
            self.move(offset, need_fit=need_fit)
            for i in range(ENGINE_NUM):
                self.engine_model[i][self.point_count] = self.last_engine_list[i]
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def addparabuttoncmd(self):
        offset = NINE_POINT[self.point_count]

        # 向量长度
        offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
        self.len_params.append(offset_len)

        for i in range(ENGINE_NUM):
            self.engine_real[i][self.point_count] = self.last_engine_list[i]
        LOG.info(f"engine_real: {self.engine_real.T}")

        self.rectext.config(state=tk.NORMAL)
        self.rectext.insert(tk.END, "成功添加数据" + "\n")
        self.rectext.config(state=tk.DISABLED)
        self.rectext.yview_moveto(1)
        self.rectext.update()

        if self.point_count < POINT_NUM - 1:
            tk.messagebox.showinfo(
                title="成功添加数据",
                message=f"数据点{self.point_count}校准成功，点击确定将校准下一个点",
                parent=self.paramwindow,
            )
            self.zerobuttoncmd()
            time.sleep(5)
            self.do_next_point()

        else:
            self.calcbuttoncmd()
            tk.messagebox.showinfo(
                title="成功机械臂校准",
                message="所有点已经全部校准！",
                parent=self.paramwindow,
            )
            self.reset()

    def calcbuttoncmd(self):
        # 三线性线性插值法
        # - rbf: 径向基函数（适合光滑的误差场）
        # - idw: 反距离权重（适合局部变化）
        target_coords = self.engine_model.T.tolist()  # 期望到达的位置
        actual_coords = self.engine_real.T.tolist()  # 校准补偿后的位置

        # 2. 训练校准模型
        calibrator = TrilinearCalibrator(grid_resolution=10)
        stats = calibrator.train(target_coords, actual_coords)
        print(stats)

        calibrator.save_model(ROOT + "/calibration_rbf_model.pickle")
        calibrator.plot()

        # # 线性插值法
        # # 边界值处理，设置长度小于最小跟大于最长的值
        # self.len_params.append(0)
        # self.len_params.append(500)
        # self.len_params = np.array(self.len_params)

        # for i in range(ENGINE_NUM):
        #     engine_err = self.engine_real[i] - self.engine_model[i]
        #     LOG.info(f"engine_real{i}: {self.engine_real[i]}")  # after fit
        #     LOG.info(f"engine_model{i}: {self.engine_model[i]}")  # before fit

        #     new_engine_err = np.append(engine_err, [engine_err[1], engine_err[8]])
        #     fit = FunctionFitter(self.len_params, new_engine_err)

        #     fit.save(ROOT + "/calibration" + str(i) + ".pickle")
        #     fit.plot()
        self.load_fit()

    def load_fit(self):
        self.calibrator = TrilinearCalibrator.load_model(
            ROOT + "/calibration_rbf_model.pickle.npz"
        )
        # self.loaded_fit0 = FunctionFitter.load(ROOT + "/calibration0.pickle")
        # self.loaded_fit1 = FunctionFitter.load(ROOT + "/calibration1.pickle")
        # self.loaded_fit2 = FunctionFitter.load(ROOT + "/calibration2.pickle")

    def send_last_send(self):
        self.move(self.last_engine_list, t=50, need_fit=False)

        self.rectext.config(state=tk.NORMAL)
        self.rectext.insert(
            tk.END, ",".join([str(s) for s in self.last_engine_list]) + "\n"
        )
        self.rectext.config(state=tk.DISABLED)
        self.rectext.yview_moveto(1)
        self.rectext.update()

    def addbutton0cmd(self):
        self.last_engine_list[0] = self.last_engine_list[0] + int(self.stepinp.get())
        self.send_last_send()

    def addbutton1cmd(self):
        self.last_engine_list[1] = self.last_engine_list[1] + int(self.stepinp.get())
        self.send_last_send()

    def addbutton2cmd(self):
        self.last_engine_list[2] = self.last_engine_list[2] + int(self.stepinp.get())
        self.send_last_send()

    def reducebutton0cmd(self):
        self.last_engine_list[0] = self.last_engine_list[0] - int(self.stepinp.get())
        self.send_last_send()

    def reducebutton1cmd(self):
        self.last_engine_list[1] = self.last_engine_list[1] - int(self.stepinp.get())
        self.send_last_send()

    def reducebutton2cmd(self):
        self.last_engine_list[2] = self.last_engine_list[2] - int(self.stepinp.get())
        self.send_last_send()

    def zerobuttoncmd(self):
        # Y轴含工具延长部分
        # 先进行原点查找，再动到初始位置（INIT_ENGINE_Y），如果此时机械臂实际未移动，说明已经在原点（意味着此时工具长度已被设置成TOOL_LEN）
        self.last_engine_list = [0, INIT_ENGINE_Y, INIT_ENGINE]
        self.send_last_send()

    def initposbuttoncmd(self):
        self.reset()

    def curposbuttoncmd(self):
        self.position()

    def baudrateselectcmd(self, *args):
        self.baudrate = int(self.baudrateselect.get())
        self.serial.baudrate = self.baudrate
        pass

    def armselectcmd(self, *args):
        self.arm = self.armselect.get()
        pass

    def serialselectcmd(self, *args):
        self.device = self.serialselect.get().split()[0]
        self.serial.port = self.device
        pass

    def search(self):
        self.devices = []
        self.comports = list(serial.tools.list_ports.comports())
        for comport in self.comports:
            self.devices.append(comport.device)
            pass
        pass

    def update(self):
        while self.thread_open_flag:
            time.sleep(1)
            try:
                if self.connecting == False:
                    self.search()
                    self.serialselect["value"] = self.comports
                    if len(list(self.serialselect["value"])) == 0:
                        self.serialselect["value"] = [""]
                        self.serialselect.current(0)
                        self.device = None
                        pass
                    elif self.device == None or self.device not in self.devices:
                        self.serialselect.current(0)
                        self.device = self.devices[0]
                        pass
                    self.serialselect.update()
                    self.face.update_idletasks()
                    pass
                pass
            except:
                pass
        pass

    def serialopen(self):
        try:
            self.serial = serial.Serial(
                self.device,
                baudrate=self.baudrate,
                timeout=1,
            )

            LOG.info(f"baudrate: {self.serial.baudrate}")
            LOG.info(f"port: {self.device}")
            # self.serialclose()
            # time.sleep(0.1)
            # self.serial.open()
        except Exception as error:
            tk.messagebox.showerror(
                title="无法连接到串口", message=error, parent=self.root
            )
            return False
        else:
            if self.serial.isOpen():
                LOG.info(f"device: {self.device} is connected.")
                self.connecting = True
                self.recthread = threading.Thread(target=self.receive)
                self.recthread.setDaemon(True)
                self.recthread.start()

                # serialread_thread = threading.Thread(target=self.serialread)
                # serialread_thread.daemon = True  # 主线程退出时，线程自动结束
                # serialread_thread.start()
                return True
            else:
                return False
            pass
        pass

    def serialclose(self):
        self.connecting = False
        time.sleep(0.1)
        self.serial.close()
        pass

    def receive(self):
        hexdisplay = False
        text = ""
        while self.connecting:
            try:
                nchar = self.serial.inWaiting()
                pass
            except:
                self.connecting = False
                self.serialclose()
                self.openbutton["text"] = "连接机械臂"
                pass
            if nchar:
                # LOG.debug(f"begin read")
                data = self.serial.read(nchar)
                LOG.debug(f"serial read data: {data}, len: {nchar}")

                text += data.decode(self.encoding)
                if text.count("\n") == 0:  # 未接收完整信息
                    continue
                if hexdisplay == True:
                    convert = "0123456789ABCDEF"
                    text = ""
                    for char in data:
                        text += convert[char // 16] + convert[char % 16] + " "
                        pass

                self.display_queue.put(lambda: self._update_ui_text(self.rectext, text))
                # self.root.after_idle(self._update_ui_text, text)

                if self.working__flag and text.count("ok"):
                    LOG.debug(f"command_completed_event.set")
                    self.command_completed_event.set()  # 触发事件
                text = ""
            time.sleep(0.1)
            pass
        pass

    def _update_ui_text(self, ui, text):
        try:
            ui.config(state=tk.NORMAL)
            ui.insert(tk.END, text)
            ui.config(state=tk.DISABLED)
            ui.yview_moveto(1)
            ui.update()
            # LOG.debug(f"insert rectext: {text}")
        except:
            import traceback

            traceback.print_exc()
        pass

    # 按钮
    def openbuttoncmd(self):
        if self.openbutton["text"] == "连接机械臂":
            is_open = self.serialopen()
            if is_open:
                self.openbutton["text"] = "断开机械臂"

                time.sleep(2)  # 等待机械臂初始化完成
                self.reset()
                # self.init_params() # 重启后才会生效

                pass
            pass
        else:
            # self.restoration()
            self.serialclose()
            self.openbutton["text"] = "连接机械臂"
            pass
        pass

    def serialread(self):
        self.serial.timeout = 1
        while self.serial.isOpen():
            try:
                data = self.serial.readall()
                LOG.debug(f"serial read data: {data}")
                if data:
                    data = data.decode(self.encoding)
                    self.rectext.config(state=tk.NORMAL)
                    self.rectext.insert(tk.END, data)
                    self.rectext.config(state=tk.DISABLED)
                    self.rectext.yview_moveto(1)
                    self.rectext.update()
                    pass
            except serial.SerialTimeoutException as e:
                pass
            except Exception as e:
                LOG.error(f"serial read error: {e}")
                break
            time.sleep(0.01)
        return

    def locatebuttoncmd(self):
        if self.locatebutton["text"] == "棋盘定位":
            if not self.serial.isOpen():
                tk.messagebox.showerror(
                    title="错误", message="请先连接机械臂", parent=self.root
                )
                return

            self.locatebutton["text"] = "结束定位"

            self.locatewindow = tk.Toplevel(self.face)
            self.locatewindow.title("棋盘定位")
            self.locatewindow.geometry("650x500")
            self.locatewindow.protocol("WM_DELETE_WINDOW", self.close_locatewindow)

            chesslocatebutton = tk.Button(
                self.locatewindow,
                text="象棋棋盘定位",
                command=self.chesslocatebuttoncmd,
            )
            chesslocatebutton.grid(
                row=0, column=1, ipadx=40, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )

            gobanglocatebutton = tk.Button(
                self.locatewindow,
                text="五子棋棋盘定位",
                command=self.gobanglocatebuttoncmd,
            )
            gobanglocatebutton.grid(
                row=1, column=1, ipadx=40, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )

            a4locatebutton = tk.Button(
                self.locatewindow, text="A4工作台定位", command=self.a4locatebuttoncmd
            )
            a4locatebutton.grid(
                row=2, column=1, ipadx=40, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )

            label = tk.Label(self.locatewindow, text="自定义工作台尺寸:")
            label.grid(
                row=3, column=0, ipadx=20, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )
            inp = tk.Entry(self.locatewindow)
            inp.insert(0, "200,300")
            inp.grid(
                row=3, column=1, ipadx=20, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )
            self.inplocate = inp

            customlocatebutton = tk.Button(
                self.locatewindow,
                text="自定义工作台定位",
                command=self.customlocatebuttoncmd,
            )
            customlocatebutton.grid(
                row=3, column=2, ipadx=20, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )

            label1 = tk.Label(self.locatewindow, text="取子点坐标:")
            label1.grid(
                row=4, column=0, ipadx=20, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )
            inp1 = tk.Entry(self.locatewindow)
            inp1.insert(0, "50,200,0")
            inp1.grid(
                row=4, column=1, ipadx=20, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )
            self.inpfetch = inp1

            fetchlocatebutton = tk.Button(
                self.locatewindow, text="取子点定位", command=self.fetchlocatebuttoncmd
            )
            fetchlocatebutton.grid(
                row=4, column=2, ipadx=40, ipady=15, padx=20, pady=20, sticky=("e", "w")
            )

        else:
            self.close_locatewindow()

    def close_locatewindow(self):
        self.locatewindow.destroy()
        self.locatebutton["text"] = "棋盘定位"

    def customlocatebuttoncmd(self):
        try:
            size = list(map(float, self.inpfetch.get().split(",")))
            wide, length = size[0], size[1]
            if length > 360:
                tk.messagebox.showerror(
                    title="输入错误",
                    message="输入错误或尺寸超出机械臂约束",
                    parent=self.locatewindow,
                )
                return
            self.location(100 + wide / 2, length / 2)
        except:
            tk.messagebox.showerror(
                title="输入错误",
                message="输入错误或尺寸超出机械臂约束",
                parent=self.locatewindow,
            )

    def chesslocatebuttoncmd(self):
        # 120为棋盘与机械臂原点x轴的距离，100为棋盘的宽度/2（x轴方向），100为棋盘的长度/2（y轴方向）
        self.location(120 + 100, 100, mid=True)

    def gobanglocatebuttoncmd(self):
        # 75为棋盘与机械臂原点x轴的距离，147为棋盘的宽度/2（x轴方向），145为棋盘的长度/2（y轴方向）
        self.location(75 + 147, 145)

    def a4locatebuttoncmd(self):
        # 100为A4纸与机械臂原点x轴的距离，148.5为棋盘的宽度/2（x轴方向），105为A4纸的长度/2（y轴方向）
        self.location(100 + 148.5, 105)

    def location(self, distance, length, mid=False):
        self.move([distance, length, 10])
        time.sleep(1)
        self.move([distance, length, -5])
        time.sleep(0.5)
        self.move([distance, length, 50])
        if mid:  # 中间点
            self.move([118, 0, 50])
            self.move([118, 0, 10])
            time.sleep(1)
            self.move([118, 0, -5])
            time.sleep(0.5)
            self.move([118, 0, 50])
        self.move([distance, -length, 50])
        self.move([distance, -length, 10])
        time.sleep(1)
        self.move([distance, -length, -5])
        time.sleep(0.5)
        self.move([distance, -length, 50])
        self.reset()

    def fetchlocatebuttoncmd(self):
        pick_point_gobang = list(map(float, self.inpfetch.get().split(",")))

        self.move(
            [pick_point_gobang[0], pick_point_gobang[1] - 50, pick_point_gobang[2] + 20]
        )
        self.move(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] + 20]
        )
        time.sleep(1)
        self.move(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] - 5]
        )
        time.sleep(1)
        self.move(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] + 50]
        )
        self.move(
            [pick_point_gobang[0], pick_point_gobang[1] - 50, pick_point_gobang[2] + 50]
        )

        hangler = YamlHandler(ROBOT_PARAMS)
        data = hangler.read_yaml()
        LOG.debug(f"yaml修改前数据：{data}")
        data["pick_point"] = pick_point_gobang
        hangler.write_yaml(data)
        LOG.debug(f"yaml修改后数据：{data}")

        self.reset()

    def runbuttoncmd(self):
        if self.runbutton["text"] == "开启指令监听":
            if not self.serial.isOpen():
                tk.messagebox.showerror(
                    title="无法抓取", message="请先连接机械臂", parent=self.root
                )
                return
            # 开启线程循环等待下棋指令、移动机械臂
            t1 = threading.Thread(target=self.working)
            t1.setDaemon(True)
            t1.start()
            self.working__flag = True
            self.runbutton["text"] = "停止指令监听"
        else:
            self.working__flag = False
            self.runbutton["text"] = "开启指令监听"
            self.reset()
            self.server.close()

    def parse_gcode(self, gcode_string):
        """
        解析G代码字符串中的所有参数

        参数:
            gcode_string (str): G代码字符串

        返回:
            dict: 包含所有参数的字典
        """
        # 匹配字母后跟数字的模式
        pattern = r"([A-Z])([-+]?\d*\.?\d+)"
        matches = re.findall(pattern, gcode_string)

        parameters = {}
        for param, value in matches:
            parameters[param] = int(value)
        print(parameters)
        return parameters

    # 读取界面参数控制机械臂
    def sendbuttoncmd(self):
        if self.connecting:
            data = self.sendtext.get(1.0, tk.END)
            # print(data.encode("utf-8").hex())

            cmds = data.split("\n")  # 按行分割
            print(cmds)
            for cmd in cmds:
                if cmd.strip() == "":
                    continue
                if not cmd.endswith("\r"):
                    cmd = cmd + "\r"
                LOG.debug(f"发送给机械臂的命令：{cmd}")
                if cmd.startswith("M"):
                    self.serial.write(cmd.encode(self.encoding))
                elif cmd.startswith("G"):
                    params = self.parse_gcode(cmd)
                    if "X" in params and "Y" in params and "Z" in params:
                        self.move(
                            [params["X"], params["Y"], params["Z"]], need_fit=False
                        )
                    else:
                        LOG.error(f"命令格式不正确！")
                else:
                    tk.messagebox.showerror(
                        title="无法发送",
                        message="命令格式不正确！",
                        parent=self.root,
                    )
                    pass
        else:
            tk.messagebox.showerror(
                title="无法发送", message="请先连接机械臂", parent=self.root
            )
            pass
        pass

    ############################ 机械臂控制函数 ##################################
    # 初始化参数
    def set_params(self):
        if not self.serial.isOpen():
            return
        data = "M210 S69 \r"  # 重启后才会生效
        self.serial.write(data.encode(self.encoding))

    # 复位
    def reset(self):
        if not self.serial.isOpen():
            return
        # Default ABSOLUTE MODE
        if self.arm == "openarm":
            # 机械臂位置查找
            data = "G28\r"
        elif self.arm == "uarm":
            # M2400 S0 : 常规模式 末端执行器 : 吸盘
            # uarm 通过传感器可以知道机械臂位置，无需复位
            data = "M2400 S0\n"
        self.serial.write(data.encode(self.encoding))

    def position(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M114\r"
        elif self.arm == "uarm":
            data = "P2220\n"

        self.serial.write(data.encode(self.encoding))

    # 气泵吸
    def suckup(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M22 \r"
        elif self.arm == "uarm":
            data = "M2231 V1 \r"

        # data = "M5\r" #scaral机器人
        self.serial.write(data.encode(self.encoding))

    # 气泵放
    def suckdown(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M21 \r"
        elif self.arm == "uarm":
            data = "M2231 V0 \r"
        self.serial.write(data.encode(self.encoding))

    #  gripper on
    def gripperon(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M3 \r"
        elif self.arm == "uarm":
            data = "M2232 V1 \r"
        self.serial.write(data.encode(self.encoding))

    # gripper off
    def gripperoff(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M5 \r"
        elif self.arm == "uarm":
            data = "M2232 V0 \r"
        self.serial.write(data.encode(self.encoding))

    # 12V直流输出
    def dc_level_on(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M31 \r"
        elif self.arm == "uarm":
            data = "M2240 N1 V1 \r"  # 未测试

        self.serial.write(data.encode(self.encoding))

    def dc_level_off(self):
        if not self.serial.isOpen():
            return
        if self.arm == "openarm":
            data = "M32 \r"
        elif self.arm == "uarm":
            data = "M2240 N1 V0 \r"  # 未测试
        self.serial.write(data.encode(self.encoding))

    def pause(self):
        self.pause_flag = True
        # data = "$DST!\n"
        # self.serial.write(data.encode(self.encoding))  # 多发一次
        LOG.debug("机械臂已经暂停工作")

    def resume(self):
        self.pause_flag = False
        # data = "$DST!\n"
        # self.serial.write(data.encode(self.encoding))  # 多发一次
        LOG.debug("机械臂已经恢复工作")

    # 机械臂运动
    def move(self, offset, t=50, need_fit=True):
        # Scara机械臂的运动逆解是下位机完成的，这里直接发送坐标即可
        engine0, engine1, engine2 = offset
        if need_fit:  # and self.arm == "openarm":
            engine0, engine1, engine2 = self.calibrator.calibrate(offset)
            engine0 = engine0 + MANUAL_COMPENSATE_X
            engine1 = engine1 + MANUAL_COMPENSATE_Y
            engine2 = engine2 + MANUAL_COMPENSATE_Z

        engine0, engine1, engine2 = round(engine0), round(engine1), round(engine2)
        LOG.debug(f"engine0:{engine0},engine1:{engine1},engine2:{engine2}")

        if not self.serial.isOpen():
            LOG.error("机械臂未连接")
            return

        # SCARA机械臂直接发送坐标
        data = (
            "G0"
            + " X"
            + str(engine0)
            + " Y"
            + str(engine1)
            + " Z"
            + str(engine2)
            + " F"
            + str(t)
            + "\r"
        )
        self.serial.write(data.encode(self.encoding))
        LOG.debug(f"send data: {data}")
        self.last_engine_list = [engine0, engine1, engine2]

    def working(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 初始化
        self.server.bind(SERVER_ADDR)  # 绑定ip和端口

        self.server.listen(5)  # 监听，设置最大数量是5
        self.pause_flag = False
        self.last_command = ""

        self.command_queue = queue.Queue()
        _thread = threading.Thread(target=self.deal_command)
        _thread.setDaemon(True)
        _thread.start()

        LOG.debug("----开始等待接受客户端下棋指令----")
        while self.working__flag:
            try:
                self.conn, addr = self.server.accept()  # 获取客户端地址
                LOG.debug(f"客户端来数据了,地址:{addr}")
                while self.working__flag:
                    try:
                        LOG.debug(f"等待客户端数据中...")
                        data = self.conn.recv(1024)  # 接收数据
                        command = data.decode()

                        # self.rectext1.config(state=tk.NORMAL)
                        # self.rectext1.insert(tk.END, command + "\n")
                        # self.rectext1.config(state=tk.DISABLED)
                        # self.rectext1.yview_moveto(1)
                        # self.rectext1.update()

                        LOG.debug(f"已接收数据：{command}")
                        if not command:
                            LOG.debug("client has lost")
                            break
                        self.command_queue.put(command)
                    except:
                        break
            except:
                break

    def deal_command(self):
        while self.working__flag:
            try:
                command = self.command_queue.get()
                LOG.debug(f"开始执行指令：{command}")
                LOG.debug("command_completed_event.clear")
                self.command_completed_event.clear()
                self.do_command(command)
                LOG.debug("command_completed_event.wait")
                self.command_completed_event.wait()  # 阻塞等待命令完成
            except:
                import traceback

                traceback.print_exc()
                break
            time.sleep(0.01)

    def do_command(self, command, t=50):
        if self.pause_flag and "resume" not in command:
            return

        if "pause" not in command and "resume" not in command:
            self.last_command = command

        if command.startswith("move"):
            offset = command.split(",")[1:-1]  # 读取物体位置
            offset = [float(i) for i in offset]
            LOG.debug(f"坐标为：{offset}")
            if len(offset) == 3:
                self.move(offset, t)
        elif command.startswith("pick"):
            self.suckup()
        elif command.startswith("down"):
            self.suckdown()
        elif command.count("pause"):
            self.pause()
        elif command.count("resume"):
            self.resume()
        else:
            LOG.debug("未知指令错误")
            return

        if self.pause_flag == False:
            self.conn.send("done".encode())  # 返回数据


if __name__ == "__main__":
    assistant = RobotSerialPortWindow()
