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
import time
import socket
import numpy as np
from math import sqrt

from robot.tools import YamlHandler, CurveFitting
from robot.robot_ik import inverse_kinematics
from robot import SERVER_ADDR, ROOT, LOG

ROBOT_PARAMS = '/robot_params.yaml'

PER_ANGLE_TIME = 25  # 机械臂运行速度：舵机转动一度需要的时间，ms

INIT_ENGINE_0 = 1475
INIT_ENGINE_1 = 1505
INIT_ENGINE_2 = 1540


class RobotSerialPortWindow:
    def __init__(self, window_flag_bit=None):
        self.serial = serial.Serial()
        self.device = None
        self.baudrate = 9600
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
        self.engine_0 = 1500
        self.len_calibration_0 = 0
        self.len_calibration_1 = 0
        self.len_calibration_2 = 0
        self.read_params()
        self.last_angle_list = [0.0, 0.0, 0.0]
        self.last_engine_list = [INIT_ENGINE_0, INIT_ENGINE_1, INIT_ENGINE_2]
        self.per_angle_time = PER_ANGLE_TIME  # 舵机转动一度需要的时间，ms

        self.window_flag_bit = window_flag_bit
        self.window()


    def window(self):
        self.root = tk.Tk()
        self.root.title('机械臂运动控制器 V0.0.1')
        self.root.geometry('960x640')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.face = tk.Frame(self.root)
        self.face.config(height=640, width=960)
        self.face.propagate(False)
        self.face.pack(anchor='nw')

        spaceframe1 = tk.Frame(self.face)
        spaceframe1.config(height=140, width=10)
        spaceframe1.propagate(False)
        spaceframe1.pack(anchor='nw', side='left')

        textframe1 = tk.Frame(self.face)
        textframe1.config(height=140, width=725)
        textframe1.propagate(False)
        textframe1.pack(anchor='nw', side='bottom')

        self.rectext1 = tk.Text(textframe1, height=35, width=99, bg='black', fg="#00FF00")
        self.recscrollbar1 = tk.Scrollbar(textframe1)
        self.rectext1['yscrollcommand'] = self.recscrollbar1.set
        self.rectext1.config(state=tk.DISABLED)
        self.recscrollbar1['command'] = self.rectext1.yview
        self.rectext1.pack(side=tk.LEFT, fill=tk.BOTH)
        self.recscrollbar1.pack(side='left', fill=tk.Y)

        # operate frame
        operateframe = tk.Frame(self.face)
        operateframe.config(height=140, width=960)
        operateframe.propagate(False)
        operateframe.pack(anchor='nw', side='bottom')

        # send text
        operatetextframe = tk.Frame(operateframe)
        operatetextframe.config(height=140, width=725)
        operatetextframe.propagate(False)
        operatetextframe.pack(anchor='nw', side='left')

        operatespaceframe = tk.Frame(operatetextframe)
        operatespaceframe.config(height=10, width=725)
        operatespaceframe.propagate(False)
        operatespaceframe.pack(anchor='sw', side='bottom')

        # operate right
        operateframeright = tk.Frame(operateframe)
        operateframeright.config(height=150, width=225)
        operateframeright.propagate(False)
        operateframeright.pack(anchor='nw', side='left')

        # text
        self.sendtext = tk.Text(operatetextframe, height=15, width=99, bg='white', fg="black")
        self.sendscrollbar = tk.Scrollbar(operatetextframe)
        self.sendtext['yscrollcommand'] = self.sendscrollbar.set
        self.sendscrollbar['command'] = self.sendtext.yview
        self.sendtext.pack(side=tk.LEFT)
        self.sendscrollbar.pack(side='left', fill=tk.Y)

        # text frame
        textframe = tk.Frame(self.face)
        textframe.config(height=350, width=725)
        textframe.propagate(False)
        textframe.pack(anchor='nw', side='left')

        # text black
        self.rectext = tk.Text(textframe, height=35, width=99, bg='black', fg="#00FF00")
        self.recscrollbar = tk.Scrollbar(textframe)
        self.rectext['yscrollcommand'] = self.recscrollbar.set
        self.rectext.config(state=tk.DISABLED)
        self.recscrollbar['command'] = self.rectext.yview
        self.rectext.pack(side=tk.LEFT, fill=tk.BOTH)
        self.recscrollbar.pack(side='left', fill=tk.Y)

        # option frame
        optionframe = tk.Frame(self.face)
        optionframe.config(height=900., width=225)
        optionframe.propagate(False)
        optionframe.pack(anchor='ne', side='right')

        # option
        optionframebottom = tk.Frame(optionframe)
        optionframebottom.config(height=180., width=225)
        optionframebottom.propagate(False)
        optionframebottom.pack(anchor='sw', side='bottom')

        # left
        optionframeleft = tk.Frame(optionframe)
        optionframeleft.config(height=180., width=60)
        optionframeleft.propagate(False)
        optionframeleft.pack(anchor='nw', side='left')
        # right
        optionframeright = tk.Frame(optionframe)
        optionframeright.config(height=180., width=165)
        optionframeright.propagate(False)
        optionframeright.pack(anchor='nw', side='left')

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
        self.baudrateselect['value'] = [1382400, 921600, 460800, 256000, 230400, \
                                        128000, 115200, 76800, 57600, 43000, 38400, 19200, 14400, \
                                        9600, 4800, 2400, 1200]
        self.baudrateselect.current(13)
        self.baudrateselect.pack()
        # cal bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label3 = tk.Label(optionframeleft, text="校验位", width=5, height=1)
        label3.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.calbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.calbitselect['value'] = ["无校验", "奇校验", "偶校验"]
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
        self.databitselect['value'] = [8, 7, 6, 5]
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
        self.stopbitselect['value'] = [1]
        self.stopbitselect.current(0)
        self.stopbitselect.pack()

        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.openbutton = tk.Button(optionframebottom, text='连接机械臂', \
                                    width=20, height=1, command=self.openbuttoncmd)
        self.openbutton.pack()

        # remote
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.remotebutton = tk.Button(optionframebottom, text='开始遥控', \
                                      width=20, height=1, command=self.remotebuttoncmd)
        self.remotebutton.pack()

        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.runbutton = tk.Button(optionframebottom, text='开始抓取', \
                                   width=20, height=1, command=self.runbuttoncmd)
        self.runbutton.pack()

        # send botton
        spacelabel = tk.Label(operateframeright, width=5, height=1)
        spacelabel.pack()
        self.sendbutton = tk.Button(operateframeright, text='发送坐标', \
                                    width=20, height=1, command=self.sendbuttoncmd)
        self.sendbutton.pack(side='top')

        spacelabel = tk.Label(operateframeright, width=5, height=1)
        spacelabel.pack()
        self.calcparambutton = tk.Button(operateframeright, text='计算内参', \
                                         width=20, height=1, command=self.calcparam)
        self.calcparambutton.pack()

        self.root.mainloop()
        self.root.quit()

    def close(self):
        self.working__flag = False
        self.thread_open_flag = False
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 1)
        try:
            self.root.destroy()
            self.server.close()
            self.restoration()
        except:
            pass

    def calcparam(self):
        w1 = tk.Toplevel(self.face)
        w1.title("机械臂内参计算")
        w1.geometry("350x400")

        label1 = tk.Label(w1, text='目标点:')
        label1.grid(row=0, column=0)
        inp1 = tk.Entry(w1)
        inp1.insert(0, '150,0,0')
        inp1.grid(row=0, column=1)
        self.inp1 = inp1

        targetbutton = tk.Button(w1, text='移动到目标点', command=self.targetbuttoncmd)
        targetbutton.grid(row=0, column=3)

        reducebutton0 = tk.Button(w1, text='1轴减10', command=self.reducebutton0cmd)
        reducebutton0.grid(row=1, column=0, padx=20, pady=20, sticky=('e', 'w'))

        addbutton0 = tk.Button(w1, text='1轴加10', command=self.addbutton0cmd)
        addbutton0.grid(row=1, column=3, padx=20, pady=20, sticky=('e', 'w'))

        reducebutton1 = tk.Button(w1, text='1轴减10', command=self.reducebutton1cmd)
        reducebutton1.grid(row=2, column=0, padx=20, pady=20, sticky=('e', 'w'))

        addbutton1 = tk.Button(w1, text='1轴加10', command=self.addbutton1cmd)
        addbutton1.grid(row=2, column=3, padx=20, pady=20, sticky=('e', 'w'))

        reducebutton2 = tk.Button(w1, text='2轴减10', command=self.reducebutton2cmd)
        reducebutton2.grid(row=3, column=0, padx=20, pady=20, sticky=('e', 'w'))

        addbutton2 = tk.Button(w1, text='2轴加10', command=self.addbutton2cmd)
        addbutton2.grid(row=3, column=3, padx=20, pady=20, sticky=('e', 'w'))

        addparabutton = tk.Button(w1, text='添加数据', command=self.addparabuttoncmd)
        addparabutton.grid(row=4, column=1, ipadx=20, ipady=10, padx=20, pady=20, sticky=('e', 'w'))

        calcbutton = tk.Button(w1, text='计算结果', command=self.calcbuttoncmd)
        calcbutton.grid(row=5, column=1, ipadx=20, ipady=10, padx=20, pady=20, sticky=('e', 'w'))

        self.angle_params_0 = []
        self.angle_params_1 = []
        self.angle_params_2 = []

        self.len_params = []

        self.engine_params_0 = []
        self.engine_params_1 = []
        self.engine_params_2 = []

    def targetbuttoncmd(self):
        if self.connecting:
            offset = list(map(float, self.inp1.get().split(',')))
            self.robotrun(offset)
        else:
            tk.messagebox.showinfo(title='无法发送', message='请先连接机械臂')
            pass
        pass

    def addparabuttoncmd(self):
        offset = list(map(float, self.inp1.get().split(',')))
        hasik, angle0, angle1, angle2 = inverse_kinematics(offset[0], offset[1], offset[2])
        if not hasik:
            return
        offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)

        self.angle_params_0.append(angle0)
        self.angle_params_1.append(angle1)
        self.angle_params_2.append(-angle2)

        self.engine_params_0.append(self.last_engine_list[0])
        self.engine_params_1.append(self.last_engine_list[1])
        self.engine_params_2.append(self.last_engine_list[2])
        self.len_params.append(offset_len)

        self.rectext.config(state=tk.NORMAL)
        self.rectext.insert(tk.END, "成功添加数据" + '\n')
        self.rectext.config(state=tk.DISABLED)
        self.rectext.yview_moveto(1)
        self.rectext.update()

    def calcbuttoncmd(self):
        if len(self.len_params) < 4:
            tk.messagebox.showinfo(title='无法计算', message='参数过少，至少需要4个参数，但最好是6个参数')
            return
        self.angle_params_0 = np.array(self.angle_params_0)
        self.angle_params_1 = np.array(self.angle_params_1)
        self.angle_params_2 = np.array(self.angle_params_2)
        self.len_params = np.array(self.len_params)
        self.engine_params_0 = np.array(self.engine_params_0)
        self.engine_params_1 = np.array(self.engine_params_1)
        self.engine_params_2 = np.array(self.engine_params_2)

        fitter = CurveFitting()
        res = fitter.calc(self.angle_params_0, self.len_params, self.engine_params_0)
        self.write_params(res, 0)
        res = fitter.calc(self.angle_params_1, self.len_params, self.engine_params_1)
        self.write_params(res, 1)
        res = fitter.calc(self.angle_params_2, self.len_params, self.engine_params_2)
        self.write_params(res, 2)

    def write_params(self, res, num):
        LOG.debug(f"结果：{res[0]}, {res[1]}")
        hangler = YamlHandler(ROOT + ROBOT_PARAMS)
        data = hangler.read_yaml()
        data['len_calibration_' + str(num)] = float(res[0])
        data['init_engine_' + str(num)] = int(res[1])
        hangler.write_yaml(data)

    def read_params(self):
        global INIT_ENGINE_0, INIT_ENGINE_1, INIT_ENGINE_2
        data = YamlHandler(ROOT + ROBOT_PARAMS).read_yaml()
        INIT_ENGINE_0 = data['init_engine_0']
        INIT_ENGINE_1 = data['init_engine_1']
        INIT_ENGINE_2 = data['init_engine_2']
        self.len_calibration_0 = data['len_calibration_0']
        self.len_calibration_1 = data['len_calibration_1']
        self.len_calibration_2 = -data['len_calibration_2']

    def send_last_send(self):
        self.sendmsg(engine0=self.last_engine_list[0], engine1=self.last_engine_list[1],
                     engine2=self.last_engine_list[2], run_time=100)
        self.rectext.config(state=tk.NORMAL)
        self.rectext.insert(tk.END, ','.join([str(s) for s in self.last_engine_list]) + '\n')
        self.rectext.config(state=tk.DISABLED)
        self.rectext.yview_moveto(1)
        self.rectext.update()

    def addbutton0cmd(self):
        self.last_engine_list[0] = self.last_engine_list[0] + 10
        self.send_last_send()

    def addbutton1cmd(self):
        self.last_engine_list[1] = self.last_engine_list[1] + 10
        self.send_last_send()

    def addbutton2cmd(self):
        self.last_engine_list[2] = self.last_engine_list[2] + 10
        self.send_last_send()

    def reducebutton0cmd(self):
        self.last_engine_list[0] = self.last_engine_list[0] - 10
        self.send_last_send()

    def reducebutton1cmd(self):
        self.last_engine_list[1] = self.last_engine_list[1] - 10
        self.send_last_send()

    def reducebutton2cmd(self):
        self.last_engine_list[2] = self.last_engine_list[2] - 10
        self.send_last_send()

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
                    self.serialselect['value'] = self.comports
                    if len(list(self.serialselect['value'])) == 0:
                        self.serialselect['value'] = [""]
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
        self.serial.port = self.device
        self.serial.baudrate = self.baudrate
        self.serial.timeout = 2
        try:
            self.serialclose()
            time.sleep(0.1)
            self.serial.open()
        except Exception as error:
            tk.messagebox.showinfo(title='无法连接到串口', message=error)
            return False
        else:
            if self.serial.isOpen():
                self.connecting = True
                # self.recthread = threading.Thread(target=self.receive)
                # self.recthread.start()
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
        while self.connecting:
            try:
                nchar = self.serial.inWaiting()
                pass
            except:
                self.connecting = False
                self.serialclose()
                self.openbutton['text'] = '连接机械臂'
                pass
            if nchar:
                if self.hexdisplay.get() == False:
                    data = ''.encode('utf-8')
                    data = data + self.serial.read(nchar)
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
                    convert = '0123456789ABCDEF'
                    string = ''
                    for char in data:
                        string += convert[char // 16] + convert[char % 16] + ' '
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
        if self.openbutton['text'] == '连接机械臂':
            is_open = self.serialopen()
            if is_open:
                self.openbutton['text'] = '断开机械臂'
                time.sleep(0.1)
                self.restoration()
                pass
            pass
        else:
            self.restoration()
            self.serialclose()
            self.openbutton['text'] = '连接机械臂'
            pass
        pass

    def remotebuttoncmd(self):
        if self.remotebutton['text'] == '开始遥控':
            if not self.serial.isOpen():
                tk.messagebox.showinfo(title='无法遥控', message='请先连接机械臂')
                return
            self.root.bind("<Key>", self.func1)
            self.remotebutton['text'] = '结束遥控'
        else:
            self.root.unbind("<Key>")
            self.remotebutton['text'] = '开始遥控'

    def runbuttoncmd(self):
        if self.runbutton['text'] == '开始抓取':
            if not self.serial.isOpen():
                tk.messagebox.showinfo(title='无法抓取', message='请先连接机械臂')
                return
            t1 = threading.Thread(target=self.working)
            t1.setDaemon(True)
            t1.start()
            self.working__flag = True
            self.runbutton['text'] = '结束抓取'
        else:
            self.working__flag = False
            self.runbutton['text'] = '开始抓取'
            self.restoration()
            self.server.close()

    # 读取界面参数控制机械臂
    def sendbuttoncmd(self):
        if self.connecting:
            data = self.sendtext.get(1.0, tk.END)
            LOG.debug(f"发送给机械臂的命令：{data}")
            if data.startswith(':'):
                data = data.lstrip(':')
                angle = data.split(',')
                try:
                    angle_list = list(map(float, angle))
                    self.robotrun_angle(angle_list)
                except:
                    tk.messagebox.showinfo(title='无法发送', message='请正确输入命令！')
                    return

            elif data.startswith('{'):
                data = data + '\n'
                self.serial.write(data.encode(self.encoding))

            else:
                offset = data.split(',')
                try:
                    offset_list = list(map(float, offset))
                    self.robotrun(offset_list)
                except:
                    tk.messagebox.showinfo(title='无法发送', message='请正确输入命令！')
                    return
        else:
            tk.messagebox.showinfo(title='无法发送', message='请先连接机械臂')
            pass
        pass

    # 键盘事件
    def func1(self, event):
        LOG.debug("事件触发键盘输入:{0},对应的ASCII码:{1}".format(event.keysym, event.keycode))
        if event.keysym == "Up":
            self.engine_0 = self.engine_0 + 50
            if self.engine_0 > 2500:
                self.engine_0 = 2500
            data = "#000P" + str(self.engine_0) + "T0100!\n"
            self.serial.write(data[0:-1].encode(self.encoding))
        elif event.keysym == "Down":
            self.engine_0 = self.engine_0 - 50
            if self.engine_0 < 500:
                self.engine_0 = 500
            data = "#000P" + str(self.engine_0) + "T0100!\n"
            self.serial.write(data[0:-1].encode(self.encoding))

    ############################ 机械臂控制函数 ##################################
    # 复位
    def restoration(self):
        if not self.serial.isOpen():
            return
        self.sendmsg()
        self.last_angle_list = [0, 0, 0]

    # 气泵吸
    def suckup(self):
        if not self.serial.isOpen():
            return
        data = "{#004P1000T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.2)

        data = "{#003P2000T1000!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.8)

        data = "{#003P1000T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.1)

    # 气泵放
    def suckdown(self):
        if not self.serial.isOpen():
            return
        data = "{#004P2200T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.3)
        data = "{#004P1500T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.3)

    # 机械臂运动
    def robotrun(self, offset, t=0):
        hasik, angle0, angle1, angle2 = inverse_kinematics(offset[0], offset[1], offset[2])
        offset_len = sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
        angle0 = angle0 + offset_len * self.len_calibration_0
        angle1 = angle1 + offset_len * self.len_calibration_1
        angle2 = angle2 + offset_len * self.len_calibration_2
        if hasik:
            LOG.debug(f"机械臂角度 angle0:{angle0}, angle1:{angle1}, angle2:{angle2}")
            if t == 0:
                delta = np.array(self.last_angle_list) - np.array([angle0, angle1, angle2])
                delta = map(abs, delta)
                t = int(max(delta) * self.per_angle_time)
                if t > 4000:
                    t = 4000
            self.sendmsg(engine0=INIT_ENGINE_0 - int(angle0 * 7.28), engine1=INIT_ENGINE_1 - int(angle1 * 7.28),
                         engine2=INIT_ENGINE_2 + int(angle2 * 7.28),
                         run_time=t)
            self.last_angle_list = [angle0, angle1, angle2]
            return True
        else:
            return False

    def robotrun_angle(self, angle_list):
        angle0, angle1, angle2 = angle_list
        delta = np.array(self.last_angle_list) - np.array([angle0, angle1, angle2])
        delta = map(abs, delta)
        t = int(max(delta) * self.per_angle_time)
        if t > 4000:
            t = 4000

        self.sendmsg(engine0=int(-angle0 * 7.28) + INIT_ENGINE_0, engine1=INIT_ENGINE_1 - int(angle1 * 7.28),
                     engine2=INIT_ENGINE_2 + int(angle2 * 7.28),
                     run_time=t)
        self.last_angle_list = [angle0, angle1, angle2]

    def working(self):
        self.robotrun([110, 165, 40])  # 随便设定的初始位置，不挡住相机就行

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 初始化
        self.server.bind(SERVER_ADDR)  # 绑定ip和端口

        self.server.listen(5)  # 监听，设置最大数量是5
        self.pause_flag = False
        self.last_command = ''
        LOG.debug("开始等待接受客户端数据----")
        while self.working__flag:
            try:
                self.conn, addr = self.server.accept()  # 获取客户端地址
                LOG.debug(f"客户端来数据了,地址:{addr}")
                while self.working__flag:
                    try:
                        data = self.conn.recv(1024)  # 接收数据
                        command = data.decode()

                        self.rectext1.config(state=tk.NORMAL)
                        self.rectext1.insert(tk.END, command + '\n')
                        self.rectext1.config(state=tk.DISABLED)
                        self.rectext1.yview_moveto(1)
                        self.rectext1.update()

                        LOG.debug(f"接受的数据：{command}")
                        if not command:
                            LOG.debug("client has lost")
                            break

                        if command.count('pause'):
                            self.pause()

                        elif command.count('resume'):
                            self.resume()
                        else:
                            safe_thread = threading.Thread(target=self.do_command, args=(command,))
                            safe_thread.setDaemon(True)
                            safe_thread.start()
                    except:
                        break
            except:
                break

    def pause(self):
        self.pause_flag = True
        data = "$DST!\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.1)
        self.serial.write(data[0:-1].encode(self.encoding))  # 多发一次
        LOG.debug("机械臂已经暂停工作")

    def resume(self):
        self.pause_flag = False
        LOG.debug("机械臂已经恢复工作")
        self.do_command(self.last_command, 2000)

    def do_command(self, command, t=0):
        self.last_command = command
        if self.pause_flag:
            return
        if command.startswith('move'):
            offset = command.split(',')[1:-1]  # 读取物体位置
            offset = [int(float(i)) for i in offset]
            LOG.debug(f"坐标为：{offset}")
            if len(offset) == 3:
                self.robotrun(offset, t)
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

        elif command.startswith('pick'):
            self.suckup()
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

        elif command.startswith('down'):
            self.suckdown()
            if self.pause_flag == False:
                self.conn.send("done".encode())  # 返回数据

    # 串口发送指令到机械臂
    def sendmsg(self, engine0=INIT_ENGINE_0, engine1=INIT_ENGINE_1, engine2=INIT_ENGINE_2, engine3=1500, run_time=1500):
        if not self.serial.isOpen():
            return
        data = "{#000P" + str(engine0) + "T" + str(run_time) + "!" + \
               "#001P" + str(engine1) + "T" + str(run_time) + "!" + \
               "#002P" + str(engine2) + "T" + str(run_time) + "!" + \
               "#003P" + str(engine3) + "T" + str(run_time) + "!" + "}\n"
        # LOG.debug(data.encode(self.encoding))
        self.serial.write(data.encode(self.encoding))
        time.sleep(run_time / 1000.0 + 0.2)
        self.last_engine_list = [engine0, engine1, engine2]


if __name__ == '__main__':
    assistant = RobotSerialPortWindow()
