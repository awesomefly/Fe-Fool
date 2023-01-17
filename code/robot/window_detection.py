# -*- coding: utf-8 -*-
import time

import cv2
import os
import tkinter
from tkinter import ttk, filedialog
from PIL import Image, ImageTk
import numpy as np
import threading

from yolov5.detect_self import YoloDetecter
from robot import robot_master, LOG, ROOT
from robot.tools import Observable, get_cameras, YamlHandler, GlobalVar
from robot.image_find_focus import FocusFinder

MODEL_PATH = ROOT + '../yolov5/runs/train/exp/weights/best.pt'


def yolo_to_pixel(yolo_list, rows_b, cols_b):
    data = []
    for x, y, w, h, c in yolo_list:
        pixel_y = y * cols_b
        pixel_x = x * rows_b
        data.append([pixel_x, pixel_y, c])
    return data


def tk_show_img(panel, img):
    # img = cv2.pyrDown(img)
    cv2image = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # 转换颜色从BGR到RGBA
    current_image = Image.fromarray(cv2image)  # 将图像转换成Image对象
    imgtk = ImageTk.PhotoImage(image=current_image)
    panel.pyimage1 = imgtk
    panel.config(image=imgtk)
    panel.update()


class DetecterWindow(Observable):
    def __init__(self):
        super().__init__()
        self.detect_flag = False
        self.connect_flag = False
        self.robot_working_flag = False
        self.self_yolo = None
        self.capture = None
        self.canny_min_threshold = 30
        self.canny_max_threshold = 250

        self.window()

    def window(self):
        self.root = tkinter.Tk()
        self.root.config(width=400, height=1000)
        self.root.title("下棋&抓取")
        self.panel = tkinter.Label(self.root)
        self.panel.grid(row=0, column=1, sticky=('e', 'w'))
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.path = tkinter.StringVar(self.root)
        self.path.set(os.path.abspath(MODEL_PATH))

        self.path_label = tkinter.Label(self.root, text="模型路径:")
        self.path_label.grid(row=1, column=0)
        self.path_entry = tkinter.Entry(self.root, textvariable=self.path, state="readonly")
        self.path_entry.grid(row=1, column=1, ipadx=200)

        self.patth_button = tkinter.Button(self.root, text="路径选择", command=self.select_path)
        self.patth_button.grid(row=1, column=2)

        self.label1 = tkinter.Label(self.root, text='请输入检测设备(GPU输入0,CPU输入cpu):')
        self.inp1 = tkinter.Entry(self.root)
        self.inp1.insert(0, "0")
        self.label1.grid(row=2, column=0)
        self.inp1.grid(row=2, column=1)

        self.model_button = tkinter.Button(self.root, text="模型确认", command=self.load_model)
        self.model_button.grid(row=3, column=1)

        self.camera_label = tkinter.Label(self.root, text="请选择相机")
        self.cameraselect = ttk.Combobox(self.root)
        self.cameraselect.bind("<<ComboboxSelected>>", self.select_camera)
        self.cameraselect['value'] = get_cameras()

        self.detect_button = tkinter.Button(self.root, text='开始检测', command=self.start_detect_cmd)
        self.min_threshold_label = tkinter.Label(self.root, bg='#9FB6CD', width=18, text='')
        self.min_threshold_scale = tkinter.Scale(self.root,
                                                 label='如画面明显跳跃，请调整该值',
                                                 from_=1,
                                                 to=80,
                                                 orient=tkinter.HORIZONTAL,  # 设置Scale控件平方向显示
                                                 length=400,
                                                 tickinterval=10,  # 设置刻度滑动条的间隔
                                                 command=self.set_canny_min_threshold)  # 调用执行函数，是数值显示在 Label控件中
        self.max_threshold_label = tkinter.Label(self.root, bg='#9FB6CD', width=18, text='')
        self.max_threshold_scale = tkinter.Scale(self.root,
                                                 label='如画面明显跳跃，请调整该值',
                                                 from_=130,
                                                 to=255,
                                                 orient=tkinter.HORIZONTAL,  # 设置Scale控件平方向显示
                                                 length=400,
                                                 tickinterval=10,  # 设置刻度滑动条的间隔
                                                 command=self.set_canny_max_threshold)  # 调用执行函数，是数值显示在 Label控件中
        self.connect_button = tkinter.Button(self.root, text='开始工作', command=self.connect_cmd)

        # 工作类型
        self.game_mode = tkinter.IntVar(self.root)

        self.root.mainloop()

    def select_camera(self, *args):
        self.camera_label.grid_forget()
        self.cameraselect.grid_forget()
        self.detect_button.grid(row=3, column=1)

        dev = int(self.cameraselect.get())
        self.open_camera(dev)

    def open_camera(self, dev):
        self.capture = cv2.VideoCapture(dev, cv2.CAP_DSHOW)
        # 设置分辨率
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.detect_flag = True
        while self.detect_flag:
            img = self.get_video_frame()
            tk_show_img(self.panel, img)

    def get_video_frame(self):
        # 摄像头读取,ret为是否成功打开摄像头,true,false。 frame为视频的每一帧图像
        ret, frame = self.capture.read()
        if not ret:
            tkinter.messagebox.showinfo('错误', '摄像头无数据')
        return frame

    def close(self):
        self.detect_flag = False
        self.root.destroy()

    def connect_cmd(self):
        if self.connect_button['text'] == '开始工作':
            self.min_threshold_scale.grid_forget()
            self.min_threshold_label.grid_forget()
            self.max_threshold_scale.grid_forget()
            self.max_threshold_label.grid_forget()
            if self.game_mode.get() == 0:
                tkinter.messagebox.showinfo('提示', '未选择模式')
                return

            if self.connect_robot(self.game_mode.get()):
                self.connect_button['text'] = "返回选择其他模式"
            else:
                tkinter.messagebox.showinfo('提示', '未开启机械臂')
        else:
            self.disconnect_robot()
            self.connect_button['text'] = '开始工作'

    def connect_robot(self, game_mode):
        if self.connect_flag:
            return False
        if game_mode == 1:
            self.robot_master = robot_master.GobangRobotMaster()
        elif game_mode == 2:
            self.robot_master = robot_master.ChessRobotMaster()
        elif game_mode == 3:
            self.robot_master = robot_master.GrabRobotMaster()

        if self.robot_master.connect_robot() == 0:  # 连接成功
            self.connect_flag = True

            self.radio_button_gobang.grid_remove()
            self.radio_button_chess.grid_remove()
            self.radio_button_grab.grid_remove()

            # 注册发布话题
            self.register(self.robot_master, "yolo_res")
            self.register(self.robot_master, "safety")
            return True
        else:
            return False

    def disconnect_robot(self):
        self.unregister(self.robot_master, "yolo_res")
        self.unregister(self.robot_master, "safety")
        self.connect_flag = False
        self.robot_master.close()
        self.radio_button_gobang.grid()
        self.radio_button_chess.grid()
        self.radio_button_grab.grid()

    def start_detect_cmd(self):
        self.detect_button.grid_forget()
        self.min_threshold_scale.grid(row=3, column=1)
        self.min_threshold_label.grid(row=4, column=1)
        self.max_threshold_scale.grid(row=5, column=1)
        self.max_threshold_label.grid(row=6, column=1)
        self.connect_button.grid(row=7, column=1)

        self.radio_button_gobang = tkinter.Radiobutton(self.root, text='五子棋', variable=self.game_mode, value=1)
        self.radio_button_gobang.grid(row=8, column=0)

        self.radio_button_chess = tkinter.Radiobutton(self.root, text='象棋', variable=self.game_mode, value=2)
        self.radio_button_chess.grid(row=8, column=1)

        self.radio_button_grab = tkinter.Radiobutton(self.root, text='物体抓取', variable=self.game_mode, value=3)
        self.radio_button_grab.grid(row=8, column=2)

        self.detect()

    def load_model(self):
        dir = self.path.get()
        if not dir.endswith(".pt"):
            tkinter.messagebox.showinfo('错误', '模型错误')
        else:
            if self.inp1.get() == 'cpu':
                device = 'cpu'
            else:
                from torch.cuda import is_available
                if is_available() == False:
                    tkinter.messagebox.showinfo('错误', 'cuda未安装或版本出错，不可使用GPU')
                    return
                device = 0
            self.self_yolo = YoloDetecter(weights=dir, device=device)

            self.path_label.grid_forget()
            self.path_entry.grid_forget()
            self.patth_button.grid_forget()
            self.model_button.grid_forget()
            self.label1.grid_forget()
            self.inp1.grid_forget()
            self.camera_label.grid(row=3, column=0)
            self.cameraselect.grid(row=3, column=1)

            GlobalVar.set_value('DATA_YAML_PATH', os.path.dirname(self.path.get()) + "/../data.yaml")  # 该模型对应的数据集yaml文件

    def select_path(self):
        path_ = filedialog.askopenfilename(initialdir=MODEL_PATH)
        if path_ == "":
            self.path.get()  # 当打开文件路径选择框后点击"取消" 输入框会清空路径，所以使用get()方法再获取一次路径
        else:
            path_ = path_.replace("/", "\\")  # 实际在代码中执行的路径为“\“ 所以替换一下
            self.path.set(path_)

    def set_canny_min_threshold(self, value):
        self.min_threshold_label.config(text='当前最小阈值： ' + value)
        self.canny_min_threshold = int(value)

    def set_canny_max_threshold(self, value):
        self.max_threshold_label.config(text='当前最大阈值： ' + value)
        self.canny_max_threshold = int(value)

    def detect(self):
        pre_img = self.get_video_frame()
        last_class_list = []
        focus_finder = FocusFinder()

        safe_thread = threading.Thread(target=self.safe_detect)
        safe_thread.setDaemon(True)
        safe_thread.start()

        while self.detect_flag:
            # start_time = time.time()
            cur_img = self.get_video_frame()
            diff = cv2.absdiff(cur_img, pre_img)
            max_diff = np.max(diff)
            pre_img = cur_img
            if max_diff > 120:
                LOG.debug(f"相邻两帧像素差异最大值大于一百二:{max_diff}")
                continue

            focus_image, has_res = focus_finder.find_focus(cur_img, self.canny_min_threshold,
                                                           self.canny_max_threshold)

            if has_res:
                res_img, yolo_list = self.self_yolo.detect(focus_image)
                tk_show_img(self.panel, res_img)
                pixel_list = yolo_to_pixel(yolo_list, res_img.shape[0], res_img.shape[1])

                # 安装类型排序，如果相邻两帧的检测结果相同，则认为是可信的
                pixel_list.sort(key=lambda x: x[2], reverse=False)
                new_class_list = [i[2] for i in pixel_list]
                # LOG.debug(f"目标检测结果:{new_class_list}")
                is_correct = False
                if new_class_list == last_class_list:
                    # LOG.debug(f"可信的目标检测结果:{pixel_list}")
                    is_correct = True
                last_class_list = new_class_list

                if self.connect_flag and is_correct:
                    self.robot_working_flag = True
                    self.publish("yolo_res", pixel_list, res_img.shape)
                    self.robot_working_flag = False

            # LOG.debug(f"本帧运行时间:{time.time() - start_time}")
        if self.detect_flag:
            self.close()

    def safe_detect(self):
        file_path = GlobalVar.get_value('DATA_YAML_PATH')
        data = YamlHandler(file_path).read_yaml()
        name = data['names']
        if 'hand' not in name:
            return
        hand_class = data['names'].index('hand')
        last_stop_time = 0
        pause_flag = False
        while self.detect_flag:
            hand_flag = False
            if self.robot_working_flag:
                cur_img = self.get_video_frame()
                res_img, yolo_list = self.self_yolo.detect(cur_img)
                # tk_show_img(self.panel, res_img)
                # LOG.debug(f"人手检测结果:{yolo_list}")
                for _, _, _, _, c in yolo_list:
                    if c == hand_class:
                        hand_flag = True
                        LOG.debug("人手进入工作区域")
                        break
                if hand_flag:
                    if not pause_flag:
                        self.publish("safety", "pause")
                        pause_flag = True
                    last_stop_time = time.time()
                else:
                    if last_stop_time and time.time() - last_stop_time > 1:
                        self.publish("safety", "resume")
                        pause_flag = False
                        last_stop_time = 0

            time.sleep(0.1)


if __name__ == '__main__':
    DetecterWindow()
