# -*- coding: utf-8 -*-
import time
import numpy as np
import cv2
import tkinter
from tkinter import ttk
from PIL import Image, ImageTk

from rembg import remove
from rembg.session_factory import new_session
from robot.image_find_focus import FocusFinder
from robot.tools import get_cameras,YamlHandler
from robot import LOG,PARAMS_YAML,IMAGE_DATA_PATH

DATA_NAME = YamlHandler(PARAMS_YAML).read_yaml()['data_name']  # image_data文件夹下自己的数据集文件名
BACKGROUND_INPUT_PATH = IMAGE_DATA_PATH + DATA_NAME + "/background_imgs/"
FOREGROUND_INPUT_PATH = IMAGE_DATA_PATH + DATA_NAME + "/foreground_imgs/"


def tk_show_img(panel, img):
    # img = cv2.pyrDown(img)
    cv2image = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # 转换颜色从BGR到RGBA
    current_image = Image.fromarray(cv2image)  # 将图像转换成Image对象
    imgtk = ImageTk.PhotoImage(image=current_image)
    panel.pyimage1 = imgtk
    panel.config(image=imgtk)
    panel.update()


def resize_img_keep_ratio(img, target_size):
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    old_size = img.shape[0:2]
    # ratio = min(float(target_size)/(old_size))
    ratio = min(float(target_size[i]) / (old_size[i]) for i in range(len(old_size)))
    new_size = tuple([int(i * ratio) for i in old_size])
    img = cv2.resize(img, (new_size[1], new_size[0]))
    pad_w = target_size[1] - new_size[1]
    pad_h = target_size[0] - new_size[0]
    top, bottom = pad_h // 2, pad_h - (pad_h // 2)
    left, right = pad_w // 2, pad_w - (pad_w // 2)
    img_new = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, None, (0, 0, 0))
    return img_new


def crop_image(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)[1]
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rect = cv2.minAreaRect(contours[0])
    box = cv2.boxPoints(rect)

    box = np.round(box).astype('int64')

    ((_, _), (x2, y2), _) = rect
    hight, width = x2, y2
    aim_size = np.float32([[0, 0], [width, 0], [width, hight], [0, hight]])
    raw_size = []

    for x, y in box:
        raw_size.append([x, y])

    raw_size = np.float32(raw_size)
    translate_map = cv2.getPerspectiveTransform(raw_size, aim_size)
    translate_img = cv2.warpPerspective(img, translate_map, (int(width), int(hight)))
    return translate_img


class GeneraterWindow:
    def __init__(self, root, window_flag_bit=None):
        self.detect_flag = True
        self.save_background_flag = False
        self.save_foreground_flag = False
        self.is_first_frame = True
        self.session = new_session("u2netp")

        self.window_flag_bit = window_flag_bit
        self.window(root)

    def window(self,root):
        self.root = root
        self.root.config(width=400, height=1000)
        self.root.title("数据集制作")
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.select_camera_label = tkinter.Label(self.root, text="请选择相机")
        self.select_camera_label.grid(row=0, column=1)
        self.select_camera_combobox = ttk.Combobox(self.root)
        self.select_camera_combobox.bind("<<ComboboxSelected>>", self.select_camera)
        self.select_camera_combobox['value'] = get_cameras()
        self.select_camera_combobox.grid(row=0, column=2)

        self.run_button = tkinter.Button(self.root, text='开始制作数据集', command=self.run)

        self.camera_panel = tkinter.Label(self.root)
        self.camera_panel.grid(row=0, rowspan=5, column=0)

        self.target_panel = tkinter.Label(self.root)
        self.name_input_label = tkinter.Label(self.root, text='请输入该物品的名字:', font=12)
        self.inp_name = tkinter.Entry(self.root)
        self.class_input_label = tkinter.Label(self.root, text='请输入该物品对应的类别序号:', font=12)
        self.inp_class = tkinter.Entry(self.root)

        self.save_background_button = tkinter.Button(self.root, text='保存背景', command=self.save_background_img)
        self.save_foreground_button = tkinter.Button(self.root, text='保存物体', command=self.save_foreground_img)


    def select_camera(self, *args):
        self.select_camera_label.grid_forget()
        self.select_camera_combobox.grid_forget()
        self.run_button.grid(row=5, column=0)
        dev = int(self.select_camera_combobox.get())
        self.open_camera(dev)

    def run(self):
        self.run_button.grid_forget()

        self.name_input_label.grid(row=0, column=1)
        self.inp_name.grid(row=1, column=1)
        self.class_input_label.grid(row=2, column=1)
        self.inp_class.grid(row=3, column=1)
        self.save_foreground_button.grid(row=4, column=1, ipadx=40)
        self.target_panel.grid(row=5, column=1)

        self.save_background_button.grid(row=5, column=0, ipadx=40)

        self.foreground_detection()

    def close(self):
        self.detect_flag = False
        self.is_first_frame = True
        self.root.destroy()
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 2)

    def save_background_img(self):
        self.save_background_flag = True

    def save_foreground_img(self):
        self.save_foreground_flag = True

    def remove_background(self, input_img):
        return remove(data=input_img, session=self.session)

    def open_camera(self, dev):
        self.capture = cv2.VideoCapture(dev, cv2.CAP_DSHOW)  # 0为电脑内置摄像头
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 设置分辨率
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        while self.detect_flag:
            img = self.get_video_frame()
            tk_show_img(self.camera_panel, img)

    def get_video_frame(self):
        ret, frame = self.capture.read()  # 摄像头读取,ret为是否成功打开摄像头,true,false。 frame为视频的每一帧图像
        return frame

    def foreground_detection(self):
        focus_finder = FocusFinder()
        while self.detect_flag:
            cur_img = self.get_video_frame()
            focus_image, res = focus_finder.find_focus(cur_img)
            if res:
                if self.is_first_frame:
                    self.is_first_frame = False
                    height, width = focus_image.shape[:2]
                else:
                    focus_image = cv2.resize(focus_image, (width, height))
                    tk_show_img(self.camera_panel, focus_image)
                    if self.save_background_flag == True:
                        self.save_background_flag = False
                        cv2.imwrite(BACKGROUND_INPUT_PATH + str(time.time()) + '.png', focus_image)
                        LOG.debug("背景创建成功")

                    if self.save_foreground_flag == True:
                        self.save_foreground_flag = False
                        output = self.remove_background(focus_image)
                        output = crop_image(output)
                        if output.shape[0] > 20 and output.shape[1] > 20:
                            LOG.debug(f"物体扣除成功，文件名 : {self.inp_name.get()}")
                            p_name = FOREGROUND_INPUT_PATH + str(self.inp_name.get()) + '-' + str(
                                self.inp_class.get()) + '-' + str(time.time()) + '.png'
                            cv2.imwrite(p_name, output)
                            tk_show_img(self.target_panel, output)


if __name__ == '__main__':
    root = tkinter.Tk()
    GeneraterWindow(root)
    root.mainloop()
