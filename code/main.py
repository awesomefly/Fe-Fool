# -*- coding: utf-8 -*-
from tkinter import Tk, Button,Toplevel
from multiprocessing import Value,freeze_support
import threading

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
        self.root = Tk()
        self.root.title('铁憨憨')

        Button(self.root, text='机械臂控制', command=self.button_serialport).grid(row=0, column=0, ipadx=60,
                                                                                  ipady=15, padx=20, pady=20,
                                                                                  sticky=('e', 'w'))
        Button(self.root, text='下棋&抓取', command=self.button_detecter).grid(row=1, column=0, ipadx=60,
                                                                               ipady=15, padx=20, pady=20,
                                                                               sticky=('e', 'w'))
        Button(self.root, text='制作样本', command=self.button_generater).grid(row=2, column=0, ipadx=60,
                                                                               ipady=15, padx=20, pady=20,
                                                                               sticky=('e', 'w'))
        Button(self.root, text='一键生成数据集', command=self.button_yolodata).grid(row=3, column=0, ipadx=60,
                                                                                    ipady=15, padx=20, pady=20,
                                                                                    sticky=('e', 'w'))
        Button(self.root, text='一键训练神经网络', command=self.button_train).grid(row=4, column=0, ipadx=60,
                                                                                   ipady=15, padx=20, pady=20,
                                                                                   sticky=('e', 'w'))
        self.root.mainloop()

    def button_serialport(self):
        """
        按钮触发事件，开一个线程去运行SerialPortAssistantWindow()这个窗口
        并用window_flag_bit.value去标记这个窗口正在运行，防止同时开启多个SerialPortAssistantWindow()窗口
        """
        if self.window_flag_bit.value & (1 << 1) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 1)
            serialport_process = threading.Thread(target=self.serialport)
            serialport_process.setDaemon(True)
            serialport_process.start()


    def serialport(self):
        RobotSerialPortWindow(self.window_flag_bit)

    def button_detecter(self):
        """
        同上
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)
            detecter_process = threading.Thread(target=self.detecter_run)
            detecter_process.setDaemon(True)
            detecter_process.start()

    def detecter_run(self):
        root = Toplevel(self.root)
        DetecterWindow(root, self.window_flag_bit)

    def button_generater(self):
        """
        同上
        生成功能与下棋功能都需要占用摄像头，不能同时开启
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)
            generater_process = threading.Thread(target=self.generater_run)
            generater_process.setDaemon(True)
            generater_process.start()

    def generater_run(self):
        root = Toplevel(self.root)
        GeneraterWindow(root, self.window_flag_bit)

    def button_yolodata(self):
        """
        同上
        """
        if self.window_flag_bit.value & (1 << 3) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 3)
            yolodata_process = threading.Thread(target=self.yolodata_run)
            yolodata_process.setDaemon(True)
            yolodata_process.start()

    def yolodata_run(self):
        root = Toplevel(self.root)
        YoloDataWindow(root, self.window_flag_bit)

    def button_train(self):
        """
        yolo的训练无法在子线程都完成，所以直接在主线程创建窗口
        """
        if self.window_flag_bit.value & (1 << 4) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 4)
            root = Toplevel(self.root)
            TrainModelWindow(root, self.window_flag_bit)


if __name__ == '__main__':
    freeze_support()
    MainWindow()
