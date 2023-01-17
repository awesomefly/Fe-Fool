# -*- coding: utf-8 -*-
from tkinter import Tk, Button
from multiprocessing import freeze_support, Value, Process

from robot.window_robot import RobotSerialPortWindow
from robot.window_yolo import YoloDataWindow
from robot.window_train import TrainModelWindow
from robot.window_generate import GeneraterWindow
from robot.window_detection import DetecterWindow


class MainWindow:
    def __init__(self):
        self.window_flag_bit = Value('i', 0)  # 共享内存，用位标记某个功能的窗口是否已经打开
        self.window()

    # 主窗口
    def window(self):
        root = Tk()
        root.title('铁憨憨')

        Button(root, text='机械臂控制', command=self.button_serialport).grid(row=0, column=0, ipadx=60,
                                                                        ipady=15, padx=20, pady=20,
                                                                        sticky=('e', 'w'))
        Button(root, text='下棋&抓取', command=self.button_detecter).grid(row=1, column=0, ipadx=60,
                                                                      ipady=15, padx=20, pady=20,
                                                                      sticky=('e', 'w'))
        Button(root, text='制作样本', command=self.button_generater).grid(row=2, column=0, ipadx=60,
                                                                      ipady=15, padx=20, pady=20,
                                                                      sticky=('e', 'w'))
        Button(root, text='一键生成数据集', command=self.button_yolodata).grid(row=3, column=0, ipadx=60,
                                                                        ipady=15, padx=20, pady=20,
                                                                        sticky=('e', 'w'))
        Button(root, text='一键训练神经网络', command=self.button_train).grid(row=4, column=0, ipadx=60,
                                                                      ipady=15, padx=20, pady=20,
                                                                      sticky=('e', 'w'))
        root.mainloop()

    def button_serialport(self):
        """
        按钮触发事件，开一个子进程去运行SerialPortAssistantWindow()这个窗口（每个功能一个进程，否则会很卡）
        并用window_flag_bit.value去标记这个窗口正在运行，防止同时开启多个SerialPortAssistantWindow()窗口
        """
        if self.window_flag_bit.value & (1 << 1) == 0:
            serialport_process = Process(target=self.serialport, args=(self.window_flag_bit,))
            serialport_process.daemon = True
            serialport_process.start()
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 1)

    def serialport(self, v):
        RobotSerialPortWindow()
        v.value = v.value ^ (1 << 1)

    def button_detecter(self):
        """
        同上
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            detecter_process = Process(target=self.detecter_run, args=(self.window_flag_bit,))
            detecter_process.daemon = True
            detecter_process.start()
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)

    def detecter_run(self, v):
        DetecterWindow()
        v.value = v.value ^ (1 << 2)

    def button_generater(self):
        """
        同上
        生成功能与检测功能都需要占用摄像头，不能同时开启
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            generater_process = Process(target=self.generater_run, args=(self.window_flag_bit,))
            generater_process.daemon = True
            generater_process.start()
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)

    def generater_run(self, v):
        GeneraterWindow()
        v.value = v.value ^ (1 << 2)

    def button_yolodata(self):
        """
        同上
        """
        if self.window_flag_bit.value & (1 << 3) == 0:
            yolodata_process = Process(target=self.yolodata_run, args=(self.window_flag_bit,))
            yolodata_process.daemon = True
            yolodata_process.start()
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 3)

    def yolodata_run(self, v):
        YoloDataWindow()
        v.value = v.value ^ (1 << 3)

    def button_train(self):
        """
        yolo的训练无法在子进程都完成，所以直接在主进程创建窗口
        """
        TrainModelWindow()


if __name__ == '__main__':
    freeze_support()
    MainWindow()
