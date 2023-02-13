# -*- coding: utf-8 -*-
import os
import tkinter
from tkinter import filedialog
from yolov5 import train
from robot import ROOT

MODEL_PATH = ROOT + '../yolov5/runs/train'


class TrainModelWindow:
    def __init__(self, root, window_flag_bit=None):
        self.window_flag_bit = window_flag_bit
        self.window(root)

    def window(self, root):
        self.root = root

        self.root.title("生成Yolo模型")
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.path = tkinter.StringVar(self.root)
        self.path.set(os.path.abspath(MODEL_PATH))

        self.path_label = tkinter.Label(self.root, text="预训练模型路径:")
        self.path_label.grid(row=0, column=0)
        self.path_entry = tkinter.Entry(self.root, textvariable=self.path, state="readonly")
        self.path_entry.grid(row=0, column=1, ipadx=200)

        self.patth_button = tkinter.Button(self.root, text="路径选择", command=self.select_path)
        self.patth_button.grid(row=0, column=2)

        self.label1 = tkinter.Label(self.root, text='请输入训练设备(GPU输入0,CPU输入cpu):')
        self.inp1 = tkinter.Entry(self.root)
        self.inp1.insert(0, "0")
        self.label1.grid(row=1, column=0)
        self.inp1.grid(row=1, column=1)

        self.label2 = tkinter.Label(self.root, text='请输入epochs:')
        self.inp2 = tkinter.Entry(self.root)
        self.inp2.insert(0, "30")
        self.label2.grid(row=2, column=0)
        self.inp2.grid(row=2, column=1)

        self.btn = tkinter.Button(self.root, text='开始训练模型', command=self.run)
        self.btn.grid(row=3, column=1)

    def select_path(self):
        path_ = filedialog.askopenfilename(initialdir=MODEL_PATH)
        if path_ == "":
            self.path.get()  # 当打开文件路径选择框后点击"取消" 输入框会清空路径，所以使用get()方法再获取一次路径
        else:
            path_ = path_.replace("/", "\\")  # 实际在代码中执行的路径为“\“ 所以替换一下
            self.path.set(path_)

    def run(self):
        weights_dir = self.path.get()
        device_str = self.inp1.get()
        epochs = int(self.inp2.get())

        if device_str == 'cpu':
            device = 'cpu'
        else:
            from torch.cuda import is_available
            if not is_available():
                tkinter.messagebox.showerror('错误', 'cuda未安装或版本出错，不可使用GPU', parent=self.root)
                return
            device = 0

        self.btn.grid_forget()
        tkinter.Label(self.root, text='模型训练进度:').grid(row=3, column=0)
        progressbar = tkinter.ttk.Progressbar(self.root, length=200, maximum=100)
        progressbar.grid(row=3, column=1)
        self.root.update()

        train.run(weights=weights_dir, epochs=epochs, device=device, root=self.root, progressbar=progressbar)
        self.close()

    def close(self):
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 4)
        self.root.destroy()


if __name__ == '__main__':
    root = tkinter.Tk()
    TrainModelWindow(root)
    root.mainloop()
