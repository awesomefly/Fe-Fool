# -*- coding: utf-8 -*-
from tkinter import Tk, Button, Toplevel
from multiprocessing import Value, freeze_support, Process
import threading, traceback

from windows.window_robot import RobotSerialPortWindow
from windows.window_detection import DetecterWindow
from windows.window_generate import GeneraterWindow
from windows.window_train import TrainModelWindow
from windows.window_yolo import YoloDataWindow


class MainWindow:
    def __init__(self):
        self.window_flag_bit = Value(
            "i", 0
        )  # 共享内存，用位标记某个功能的窗口是否已经打开
        self.window()

    # 主窗口
    def window(self):
        self.root = Tk()
        self.root.title("铁憨憨")
        self.root.geometry("1080x640")
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)

        Button(self.root, text="机械臂控制", command=self.button_serialport).grid(
            row=0, column=0, ipadx=60, ipady=15, padx=20, pady=20, sticky=("e", "w")
        )
        Button(self.root, text="视觉检测", command=self.button_detecter).grid(
            row=1, column=0, ipadx=60, ipady=15, padx=20, pady=20, sticky=("e", "w")
        )
        Button(self.root, text="制作样本", command=self.button_generater).grid(
            row=2, column=0, ipadx=60, ipady=15, padx=20, pady=20, sticky=("e", "w")
        )
        Button(self.root, text="生成数据集", command=self.button_yolodata).grid(
            row=3, column=0, ipadx=60, ipady=15, padx=20, pady=20, sticky=("e", "w")
        )
        Button(self.root, text="训练神经网络", command=self.button_train).grid(
            row=4, column=0, ipadx=60, ipady=15, padx=20, pady=20, sticky=("e", "w")
        )

        self.root.mainloop()

    def button_serialport(self):
        """
        按钮触发事件,开一个线程去运行SerialPortAssistantWindow()这个窗口
        并用window_flag_bit.value去标记这个窗口正在运行，防止同时开启多个SerialPortAssistantWindow()窗口
        """
        if self.window_flag_bit.value & (1 << 1) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 1)
            self.root.after(100, self.serialport)
            # serialport_process = Process(target=self.serialport)
            # serialport_process.daemon = True
            # serialport_process.start()

    def serialport(self):
        RobotSerialPortWindow(self.window_flag_bit)

    def button_detecter(self):
        """
        视觉检测->下棋
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)
            self.root.after(0, self.detecter_run)
            # detecter_process = threading.Thread(target=self.detecter_run)
            # detecter_process.setDaemon(True)
            # detecter_process.start()

    def detecter_run(self):
        # root = Toplevel(self.root)
        root = Tk()
        DetecterWindow(root, self.window_flag_bit)

    def button_generater(self):
        """
        制作样本
        生成功能与下棋功能都需要占用摄像头，不能同时开启
        """
        if self.window_flag_bit.value & (1 << 2) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 2)
            self.root.after(0, self.generater_run)
            # generater_process = threading.Thread(target=self.generater_run)
            # generater_process.setDaemon(True)
            # generater_process.start()

    def generater_run(self):
        # root = Toplevel(self.root)
        root = Tk()
        GeneraterWindow(root, self.window_flag_bit)

    def button_yolodata(self):
        """
        生成数据集
        """
        if self.window_flag_bit.value & (1 << 3) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 3)
            self.root.after(0, self.yolodata_run)
            # yolodata_process = threading.Thread(target=self.yolodata_run)
            # yolodata_process.setDaemon(True)
            # yolodata_process.start()

    def yolodata_run(self):
        # root = Toplevel(self.root)
        root = Tk()
        YoloDataWindow(root, self.window_flag_bit)

    def button_train(self):
        """
        训练神经网络
        yolo的训练无法在子线程都完成,所以直接在主线程创建窗口
        """
        if self.window_flag_bit.value & (1 << 4) == 0:
            self.window_flag_bit.value = self.window_flag_bit.value | (1 << 4)
            self.root.after(0, self.train_run)

    def train_run(self):
        # root = Toplevel(self.root)
        root = Tk()
        TrainModelWindow(root, self.window_flag_bit)


if __name__ == "__main__":
    try:
        freeze_support()
        MainWindow()
    except Exception as e:
        print(e)
        traceback.print_exc()
