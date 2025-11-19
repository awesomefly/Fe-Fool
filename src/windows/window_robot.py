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

import time
import socket
import numpy as np
from math import sqrt

from robot.tools import YamlHandler, FunctionFitter
from robot.robot_ik import inverse_kinematics
from robot import SERVER_ADDR, ROOT, LOG

ROBOT_PARAMS = ROOT + "/robot_params.yaml"

PER_ANGLE_TIME = 15  # 机械臂运行速度：舵机转动一度需要的时间，ms
INIT_ENGINE = 120
ENGINE_NUM = 3

TOOL_LENGTH = 69  # 工具长度，洗泵延长距离为 69mm
INIT_ENGINE_Y = INIT_ENGINE + TOOL_LENGTH  # 初始引擎Y轴位置

POINT_NUM = 9  # 设置校准点位数量
NINE_POINT = [
    [-100, 149, 30],
    [0, 149, 30],
    [100, 149, 30],
    [100, 199, 30],
    [0, 199, 30],
    [-100, 199, 30],
    [-100, 224, 30],
    [0, 224, 30],
    [100, 224, 30],
]


class RobotSerialPortWindow:
    def __init__(self, window_flag_bit=None):
        self.serial = serial.Serial()
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

        # serial
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label1 = tk.Label(optionframeleft, text="端口号", width=5, height=1)
        label1.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.serialselect = ttk.Combobox(optionframeright, width=15, height=5)
        self.serialselect.bind("<<ComboboxSelected>>", self.serialselectcmd)
        self.serialselect.pack()
        # baudrate
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label2 = tk.Label(optionframeleft, text="波特率", width=5, height=1)
        label2.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
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
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label3 = tk.Label(optionframeleft, text="校验位", width=5, height=1)
        label3.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.calbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.calbitselect["value"] = ["无校验", "奇校验", "偶校验"]
        self.calbitselect.current(0)
        self.calbitselect.pack()
        # data bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label4 = tk.Label(optionframeleft, text="数据位", width=5, height=1)
        label4.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.databitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.databitselect["value"] = [8, 7, 6, 5]
        self.databitselect.current(0)
        self.databitselect.pack()
        # stop bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label5 = tk.Label(optionframeleft, text="停止位", width=5, height=1)
        label5.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
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

        self.root.mainloop()

    def close(self):
        try:
            self.restoration()
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
            self.paramwindow.geometry("400x600")
            self.paramwindow.protocol("WM_DELETE_WINDOW", self.close_paramwindow)

            self.point_count = 0

            zerobutton = tk.Button(
                self.paramwindow, text="回到原点(0,189,120)", command=self.zerobuttoncmd
            )
            zerobutton.grid(row=0, column=1, padx=20, pady=20, sticky=("e", "w"))

            initposbutton = tk.Button(
                self.paramwindow, text="原点查找", command=self.initposbuttoncmd
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
                self.paramwindow, text="打开12V直流输出", command=self.DCon
            )
            DConbutton.grid(row=2, column=4, padx=20, pady=20, sticky=("e", "w"))
            DCoffbutton = tk.Button(
                self.paramwindow, text="关闭12V直流输出", command=self.DCoff
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

            self.engine_real = np.zeros((3, 9))
            self.engine_model = np.zeros((3, 9))

        else:
            self.close_paramwindow()

    def close_paramwindow(self):
        self.stop_calc()
        self.paramwindow.destroy()
        self.calcparambutton["text"] = "机械臂校准"

    def stop_calc(self):
        self.point_count = 0
        self.len_params = []
        self.engine_real = np.zeros((3, 9))
        self.engine_model = np.zeros((3, 9))

        self.restoration()

    def testninebuttoncmd(self):
        if self.connecting:
            for i in range(POINT_NUM):
                offset = NINE_POINT[i]
                self.robotrun(offset, need_fit=True)
                time.sleep(5)

            # self.stop_calc()
            # self.do_first_point(need_fit=True)
            # time.sleep(2)
            # for i in range(POINT_NUM - 1):
            #     self.do_next_point(need_fit=True)
            #     time.sleep(2)
            # self.restoration()
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def testninebuttoncmd2(self):
        if self.connecting:
            for i in range(POINT_NUM):
                offset = NINE_POINT[i]
                self.robotrun(offset, need_fit=False)
                time.sleep(5)
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def do_first_point(self, need_fit=False):
        self.stop_calc()
        if self.connecting:
            self.point_count = 0
            offset = NINE_POINT[self.point_count]
            self.robotrun(offset, need_fit=need_fit)
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
            self.robotrun(offset, need_fit=need_fit)
            for i in range(ENGINE_NUM):
                self.engine_model[i][self.point_count] = self.last_engine_list[i]
        else:
            tk.messagebox.showerror(
                title="无法发送", message="机械臂已经断开连接", parent=self.paramwindow
            )
        pass

    def addparabuttoncmd(self):
        offset = NINE_POINT[self.point_count]
        # hasik, angle0, angle1, angle2 = inverse_kinematics(
        #     offset[0], offset[1], offset[2]
        # )
        # if not hasik:
        #     return

        # 向量长度
        offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
        self.len_params.append(offset_len)

        for i in range(ENGINE_NUM):
            self.engine_real[i][self.point_count] = self.last_engine_list[i]

        self.rectext.config(state=tk.NORMAL)
        self.rectext.insert(tk.END, "成功添加数据" + "\n")
        self.rectext.config(state=tk.DISABLED)
        self.rectext.yview_moveto(1)
        self.rectext.update()

        if self.point_count < POINT_NUM - 1:
            tk.messagebox.showinfo(
                title="成功添加数据",
                message="成功添加数据，点击确定将校准下一个点",
                parent=self.paramwindow,
            )
            self.do_next_point()

        else:
            self.calcbuttoncmd()
            tk.messagebox.showinfo(
                title="成功机械臂校准",
                message="9个点已经全部校准！",
                parent=self.paramwindow,
            )
            self.restoration()

    def calcbuttoncmd(self):
        # 边界值处理，设置长度小于最小跟大于最长的值
        self.len_params.append(0)
        self.len_params.append(500)

        self.len_params = np.array(self.len_params)

        for i in range(ENGINE_NUM):
            engine_err = self.engine_real[i] - self.engine_model[i]
            LOG.info(f"engine_real{i}: {self.engine_real[i]}")
            LOG.info(f"engine_model{i}: {self.engine_model[i]}")

            new_engine_err = np.append(engine_err, [engine_err[1], engine_err[8]])
            fit = FunctionFitter(self.len_params, new_engine_err)

            fit.save(ROOT + "/calibration" + str(i) + ".pickle")
            fit.plot()
        self.load_fit()

    def load_fit(self):
        self.loaded_fit0 = FunctionFitter.load(ROOT + "/calibration0.pickle")
        self.loaded_fit1 = FunctionFitter.load(ROOT + "/calibration1.pickle")
        self.loaded_fit2 = FunctionFitter.load(ROOT + "/calibration2.pickle")

    def send_last_send(self):
        self.sendmsg(
            engine0=self.last_engine_list[0],
            engine1=self.last_engine_list[1],
            engine2=self.last_engine_list[2],
            run_time=50,
        )
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
        data = "G28\r"
        self.serial.write(data.encode(self.encoding))
        self.last_engine_list = [0, INIT_ENGINE_Y, INIT_ENGINE]  # Y轴含工具延长部分
        # self.send_last_send()

    def curposbuttoncmd(self):
        data = "M114\r"
        self.serial.write(data.encode(self.encoding))

    def baudrateselectcmd(self, *args):
        self.baudrate = int(self.baudrateselect.get())
        self.serial.baudrate = self.baudrate
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
                if hexdisplay == False:
                    data = "".encode("utf-8")
                    data = data + self.serial.read(nchar)
                    LOG.debug(f"serial read data: {data}, len: {nchar}")
                    try:
                        self.rectext.config(state=tk.NORMAL)
                        self.rectext.insert(tk.END, data.decode(self.encoding))
                        self.rectext.config(state=tk.DISABLED)
                        self.rectext.yview_moveto(1)
                        self.rectext.update()
                        pass
                    except:
                        pass
                    pass
                else:
                    data = self.serial.read(nchar)
                    LOG.debug(f"serial read data: {data}, len: {nchar}")
                    convert = "0123456789ABCDEF"
                    string = ""
                    for char in data:
                        string += convert[char // 16] + convert[char % 16] + " "
                        pass
                    self.rectext.config(state=tk.NORMAL)
                    self.rectext.insert(tk.END, string)
                    self.rectext.config(state=tk.DISABLED)
                    self.rectext.yview_moveto(1)
                    self.rectext.update()
                    pass
                pass
            pass
        pass

    # 按钮
    def openbuttoncmd(self):
        if self.openbutton["text"] == "连接机械臂":
            is_open = self.serialopen()
            if is_open:
                self.openbutton["text"] = "断开机械臂"

                # time.sleep(3)  # 等待机械臂初始化完成
                # self.restoration()
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
        self.robotrun([distance, length, 10])
        time.sleep(1)
        self.robotrun([distance, length, -5])
        time.sleep(0.5)
        self.robotrun([distance, length, 50])
        if mid:  # 中间点
            self.robotrun([118, 0, 50])
            self.robotrun([118, 0, 10])
            time.sleep(1)
            self.robotrun([118, 0, -5])
            time.sleep(0.5)
            self.robotrun([118, 0, 50])
        self.robotrun([distance, -length, 50])
        self.robotrun([distance, -length, 10])
        time.sleep(1)
        self.robotrun([distance, -length, -5])
        time.sleep(0.5)
        self.robotrun([distance, -length, 50])
        self.restoration()

    def fetchlocatebuttoncmd(self):
        pick_point_gobang = list(map(float, self.inpfetch.get().split(",")))

        self.robotrun(
            [pick_point_gobang[0], pick_point_gobang[1] - 50, pick_point_gobang[2] + 20]
        )
        self.robotrun(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] + 20]
        )
        time.sleep(1)
        self.robotrun(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] - 5]
        )
        time.sleep(1)
        self.robotrun(
            [pick_point_gobang[0], pick_point_gobang[1], pick_point_gobang[2] + 50]
        )
        self.robotrun(
            [pick_point_gobang[0], pick_point_gobang[1] - 50, pick_point_gobang[2] + 50]
        )

        hangler = YamlHandler(ROBOT_PARAMS)
        data = hangler.read_yaml()
        LOG.debug(f"yaml修改前数据：{data}")
        data["pick_point"] = pick_point_gobang
        hangler.write_yaml(data)
        LOG.debug(f"yaml修改后数据：{data}")

        self.restoration()

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
            self.restoration()
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
                        self.robotrun(
                            [params["X"], params["Y"], params["Z"]], need_fit=False
                        )
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
    def init_params(self):
        if not self.serial.isOpen():
            return
        data = "M210 S69 \r"  # 重启后才会生效
        self.serial.write(data.encode(self.encoding))

    # 复位
    def restoration(self):
        if not self.serial.isOpen():
            return

        # Default ABSOLUTE MODE
        data = "G28 \r"
        self.serial.write(data.encode(self.encoding))

        # self.sendmsg()
        # self.last_angle_list = [INIT_ENGINE, INIT_ENGINE, INIT_ENGINE]

    # 气泵吸
    def suckup(self):
        if not self.serial.isOpen():
            return
        data = "M22 \r"
        # data = "M5\r" #scaral机器人
        self.serial.write(data.encode(self.encoding))

    # 气泵放
    def suckdown(self):
        if not self.serial.isOpen():
            return
        data = "M21 \r"
        # data = "M3\r" #scaral机器人
        self.serial.write(data.encode(self.encoding))

    #  gripper on
    def gripperon(self):
        if not self.serial.isOpen():
            return
        data = "M3 \r"
        self.serial.write(data.encode(self.encoding))

    # gripper off
    def gripperoff(self):
        if not self.serial.isOpen():
            return
        data = "M5 \r"
        self.serial.write(data.encode(self.encoding))

    # 12V直流输出
    def DCon(self):
        if not self.serial.isOpen():
            return
        data = "M31 \r"
        self.serial.write(data.encode(self.encoding))

    def DCoff(self):
        if not self.serial.isOpen():
            return
        data = "M32 \r"  # 四轴
        self.serial.write(data.encode(self.encoding))

    def angle2engine(self, angle, num):
        if num == 0 or num == 1:
            return INIT_ENGINE - int(angle * 7.28)
        else:
            return INIT_ENGINE + int(angle * 7.28)

    # 机械臂运动
    def robotrun(self, offset, t=50, need_fit=True):
        # SCARA机械臂的运动逆解是下位机完成的，这里直接发送坐标即可
        engine0 = offset[0]
        engine1 = offset[1]
        engine2 = offset[2]

        if need_fit:
            # 向量长度
            offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
            # 坐标纠偏
            engine0 = offset[0] + self.loaded_fit0.f(offset_len)
            engine1 = offset[1] + self.loaded_fit1.f(offset_len)
            engine2 = offset[2] + self.loaded_fit2.f(offset_len)

        self.sendmsg(engine0=engine0, engine1=engine1, engine2=engine2, run_time=int(t))

        """
        # 运动逆解
        hasik, angle0, angle1, angle2 = inverse_kinematics(
            offset[0], offset[1], offset[2]
        )
        if hasik:
            offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
            # 机械臂间隙太大，2轴超过90度需要补偿一些
            if angle2 > 88:
                angle2 = angle2 + 3
            LOG.debug(f"机械臂角度 angle0:{angle0}, angle1:{angle1}, angle2:{angle2}")

            if t == 0:
                delta = np.array(self.last_angle_list) - np.array(
                    [angle0, angle1, angle2]
                )
                delta = map(abs, delta)
                t = max(delta) * self.per_angle_time
                t = t + offset_len**2 / 10000  # 越远会越抖，慢一点
                if t > 4000:
                    t = 4000
                if t < 300:
                    t = 300

            # 坐标纠偏
            engine0 = self.angle2engine(angle0, 0) + self.loaded_fit0.f(offset_len)
            engine1 = self.angle2engine(angle1, 1) + self.loaded_fit1.f(offset_len)
            engine2 = self.angle2engine(angle2, 2) + self.loaded_fit2.f(offset_len)
            # LOG.debug(f"机械臂参数 engine0:{engine0}, engine1:{engine1}, engine2:{engine2}")
            self.sendmsg(
                engine0=engine0, engine1=engine1, engine2=engine2, run_time=int(t)
            )
            self.last_angle_list = [angle0, angle1, angle2]
            return True
        else:
            return False
        """

    def robotrun_angle(self, angle_list):
        angle0, angle1, angle2 = angle_list
        delta = np.array(self.last_angle_list) - np.array([angle0, angle1, angle2])
        delta = map(abs, delta)
        t = int(max(delta) * self.per_angle_time)
        if t > 4000:
            t = 4000

        self.sendmsg(
            engine0=int(-angle0 * 7.28) + INIT_ENGINE,
            engine1=INIT_ENGINE - int(angle1 * 7.28),
            engine2=INIT_ENGINE + int(angle2 * 7.28),
            run_time=t,
        )
        self.last_angle_list = [angle0, angle1, angle2]

    def working(self):
        self.robotrun([-160, 0, 120])  # 随便设定的初始位置，不挡住相机就行

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 初始化
        self.server.bind(SERVER_ADDR)  # 绑定ip和端口

        self.server.listen(5)  # 监听，设置最大数量是5
        self.pause_flag = False
        self.last_command = ""
        LOG.debug("----开始等待接受客户端下棋指令----")
        while self.working__flag:
            try:
                self.conn, addr = self.server.accept()  # 获取客户端地址
                LOG.debug(f"客户端来数据了,地址:{addr}")
                while self.working__flag:
                    try:
                        data = self.conn.recv(1024)  # 接收数据
                        command = data.decode()

                        self.rectext1.config(state=tk.NORMAL)
                        self.rectext1.insert(tk.END, command + "\n")
                        self.rectext1.config(state=tk.DISABLED)
                        self.rectext1.yview_moveto(1)
                        self.rectext1.update()

                        LOG.debug(f"接受的数据：{command}")
                        if not command:
                            LOG.debug("client has lost")
                            break

                        if command.count("pause"):
                            self.pause()

                        elif command.count("resume"):
                            self.resume()
                        else:
                            safe_thread = threading.Thread(
                                target=self.do_command, args=(command,)
                            )
                            safe_thread.setDaemon(True)
                            safe_thread.start()
                    except:
                        break
            except:
                break

    def pause(self):
        self.pause_flag = True
        data = "$DST!\n"
        self.serial.write(data.encode(self.encoding))
        time.sleep(0.1)
        self.serial.write(data.encode(self.encoding))  # 多发一次
        LOG.debug("机械臂已经暂停工作")

    def resume(self):
        self.pause_flag = False
        LOG.debug("机械臂已经恢复工作")
        self.do_command(self.last_command, 2000)

    def do_command(self, command, t=0):
        self.last_command = command
        if self.pause_flag:
            return
        if command.startswith("move"):
            offset = command.split(",")[1:-1]  # 读取物体位置
            offset = [int(float(i)) for i in offset]
            LOG.debug(f"坐标为：{offset}")
            if len(offset) == 3:
                self.robotrun(offset, t)
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

        elif command.startswith("pick"):
            self.suckup()
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

        elif command.startswith("down"):
            self.suckdown()
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

    # 串口发送指令到机械臂
    def sendmsg(
        self,
        command: str = "G1",
        engine0=INIT_ENGINE,
        engine1=INIT_ENGINE,
        engine2=INIT_ENGINE,
        engine3=INIT_ENGINE,  # 预留第四关节，暂时不启用
        run_time=1000,
    ):
        if not self.serial.isOpen():
            return

        # SCARA机械臂直接发送坐标
        data = (
            command
            + " X"
            + str(engine0)
            + " Y"
            + str(engine1)
            + " Z"
            + str(engine2)
            + " F"
            + str(run_time)
            + "\r"
        )
        self.serial.write(data.encode(self.encoding))
        LOG.debug(f"msg:{data[0:-1]}")
        self.last_engine_list = [engine0, engine1, engine2]
        return True

        """ 
        MAX_ENGINE = 2500
        MIN_ENGINE = 500
        engine0 = engine0 if engine0 <= MAX_ENGINE else MAX_ENGINE
        engine0 = engine0 if engine0 >= MIN_ENGINE else MIN_ENGINE
        engine1 = engine1 if engine1 <= MAX_ENGINE else MAX_ENGINE
        engine1 = engine1 if engine1 >= MIN_ENGINE else MIN_ENGINE
        engine2 = engine2 if engine2 <= MAX_ENGINE else MAX_ENGINE
        engine2 = engine2 if engine2 >= MIN_ENGINE else MIN_ENGINE

        data = (
            "{#000P"
            + str(int(engine0))
            + "T"
            + str(int(run_time))
            + "!"
            + "#001P"
            + str(int(engine1))
            + "T"
            + str(int(run_time))
            + "!"
            + "#002P"
            + str(int(engine2))
            + "T"
            + str(int(run_time))
            + "!"
            + "#003P"
            + str(int(engine3))
            + "T"
            + str(int(run_time))
            + "!"
            + "}\n"
        )
        # LOG.debug(data.encode(self.encoding))
        self.serial.write(data.encode(self.encoding))
        time.sleep(run_time / 1000.0 + 0.1)
        """


if __name__ == "__main__":
    assistant = RobotSerialPortWindow()
