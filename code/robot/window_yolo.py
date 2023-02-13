# -*- coding: utf-8 -*-
import tkinter
from robot import image_generate_dataset


class YoloDataWindow:
    def __init__(self, root, window_flag_bit=None):
        self.window_flag_bit = window_flag_bit
        self.window(root)

    def window(self, root):
        self.root = root
        self.root.config(width=400, height=1000)
        self.root.title("生成Yolo数据")
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.label1 = tkinter.Label(self.root, text='请输入每张背景需要生成的数据量:', font=12)
        self.inp1 = tkinter.Entry(self.root)
        self.inp1.insert(0, "5")
        self.label1.grid(row=0, column=0)
        self.inp1.grid(row=0, column=1)

        self.label2 = tkinter.Label(self.root, text='请输入前景间重叠因子(越小随机的各个前景间可重合度越高):', font=12)
        self.inp2 = tkinter.Entry(self.root)
        self.inp2.insert(0, "1")
        self.label2.grid(row=1, column=0)
        self.inp2.grid(row=1, column=1)

        self.label3 = tkinter.Label(self.root, text='请输入每个前景在背景中的随机的最大个数:', font=12)
        self.inp3 = tkinter.Entry(self.root)
        self.inp3.insert(0, "3")
        self.label3.grid(row=2, column=0)
        self.inp3.grid(row=2, column=1)

        self.label4 = tkinter.Label(self.root, text='物品是否为圆形:', font=12)
        self.var = tkinter.IntVar()
        self.var.set(1)
        self.rd1 = tkinter.Radiobutton(self.root, text="是", variable=self.var, value=1)
        self.rd2 = tkinter.Radiobutton(self.root, text="否", variable=self.var, value=0)
        self.label4.grid(row=3, column=0)
        self.rd1.grid(row=3, column=1)
        self.rd2.grid(row=3, column=2)

        self.btn = tkinter.Button(self.root, text='开始生成数据集',
                                  command=lambda: self.run(int(self.inp1.get()), float(self.inp2.get()),
                                                           float(self.inp3.get()), self.var.get() == 1))
        self.btn.grid(row=5, column=0)

    def run(self, per_num=0, overlap_factor=0, max_num=0, is_circle=True):
        self.btn.grid_forget()
        tkinter.Label(self.root, text='数据集生成进度:').grid(row=6, column=0)
        progressbar = tkinter.ttk.Progressbar(self.root, length=200, maximum=100)
        progressbar.grid(row=6, column=1)
        self.root.update()

        image_generate_dataset.run(per_num=per_num, overlap_factor=overlap_factor, max_num=max_num, is_circle=is_circle,
                                   root=self.root, progressbar=progressbar)

    def close(self):
        self.root.destroy()
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 3)


if __name__ == '__main__':
    root = tkinter.Tk()
    YoloDataWindow(root)
    root.mainloop()
