# -*- coding: utf-8 -*-
import os
from random import choice, randint
from playsound import playsound
from math import cos, sin
import numpy as np
from cv2 import VideoCapture, CAP_DSHOW,cvtColor,COLOR_BGR2RGB,COLOR_RGB2BGR
from PIL import ImageEnhance,Image
import yaml
from shutil import copy
from scipy.optimize import leastsq
from threading import Thread
from abc import ABC, abstractmethod

from robot import LOG, ROOT, IMAGE_DATA_PATH


# 跨文件全局变量
class GlobalVar(object):
    _global_dict = {}

    @staticmethod
    def set_value(key, value):
        GlobalVar._global_dict[key] = value

    @staticmethod
    def get_value(key, defValue=None):
        try:
            return GlobalVar._global_dict[key]
        except KeyError:
            return defValue


# 单例基类
class Singleton(object):
    _instance = None

    def __new__(cls, *args, **kw):
        if cls._instance is None:
            cls._instance = object.__new__(cls)
        return cls._instance


# 观察者基类
class Observer(ABC):

    @abstractmethod
    def receive_message(self, topic, message):
        pass


# 被观察者基类
class Observable:
    def __init__(self):
        self.topics = {}

    def register(self, observer, topic):
        if topic not in self.topics:
            self.topics[topic] = []
        self.topics[topic].append(observer)

    def unregister(self, observer, topic):
        if topic in self.topics:
            self.topics[topic].remove(observer)

    def publish(self, topic, *message):
        if topic in self.topics:
            for observer in self.topics[topic]:
                observer.receive_message(topic, message)


class YamlHandler:
    def __init__(self, filename):
        self.filename = filename

    def read_yaml(self):
        """读取yaml文件数据"""
        with open(self.filename, encoding='utf-8') as f:
            return yaml.load(f.read(), Loader=yaml.FullLoader)

    def write_yaml(self, data):
        """在yaml文件写入数据"""
        with open(self.filename, encoding='utf-8', mode='w') as f:
            return yaml.dump(data, stream=f, allow_unicode=True)


class CurveFitting(object):
    def __init__(self):
        self.param_init = [0, 1500]

    def fun(self, param, angle, length):  # 定义拟合函数形式
        len_calibration, angle_init = param
        return angle_init - (angle + length * len_calibration) * 11.11

    def error(self, param, angle, length, engine):  # 拟合残差
        return self.fun(param, angle, length) - engine

    def calc(self, angle_list, len_list, a):
        params = leastsq(self.error, self.param_init, args=(angle_list, len_list, a))  # 进行拟合
        LOG.info(f"拟合结果: {params[0]}")
        return params[0]


def coordinate_mapping(pixel_list, physical_rows, physical_cols, pixel_rows, pixel_cols):
    # LOG.debug(f"坐标映射: {physical_rows}, {physical_cols}, {pixel_rows}, {pixel_cols}")
    data = []
    for x, y, c in pixel_list:
        x = x * physical_rows / pixel_rows
        y = y * physical_cols / pixel_cols
        data.append([x, y, c])
    return data


def plane_coordinate_transform(coordinate_x, coordinate_y, transform_x, transform_y, transform_angle):
    transform_matrix = [[cos(transform_angle), -sin(transform_angle), transform_x],
                        [sin(transform_angle), cos(transform_angle), transform_y],
                        [0, 0, 1]]

    input_coordinate = np.array([coordinate_x, coordinate_y, 1])
    transform_matrix = np.array(transform_matrix)

    output_coordinate = np.dot(transform_matrix, input_coordinate.T)
    return output_coordinate[0], output_coordinate[1]


# 播放音频的模块会抛出异常,是由于windows不支持utf-16编码，需修改playsound源码
# 参考链接：https://blog.csdn.net/lj606/article/details/122354958
def play_sound_thread(*args):
    thread = Thread(target=play_sound, args=args)
    thread.start()


def play_sound(*args):
    SOUND_PATH = ROOT + '/src/sound/'
    for str_sound in args:
        path_list = []
        for filename in os.listdir(SOUND_PATH):
            if filename.startswith(str_sound):
                path_list.append(SOUND_PATH + filename)
        playsound(choice(path_list))


def get_name_by_class(_class):
    file_path = GlobalVar.get_value('DATA_YAML_PATH')
    data = YamlHandler(file_path).read_yaml()
    name = data['names'][_class]
    # LOG.debug(f"{name}")
    return name


# 修改透明背景为白色
def transparence_to_white(img):
    rand_threshold = randint(30, 80)
    img_new = img[:, :, 0:-1]
    transparence = img[:, :, -1]
    img_new[transparence < rand_threshold] = [255, 255, 255]
    return img_new

def random_brightness(image, min_factor=0.6, max_factor=1.3):
    image = Image.fromarray(cvtColor(image, COLOR_BGR2RGB))
    factor = np.random.uniform(min_factor, max_factor)
    image_enhancer_brightness = ImageEnhance.Brightness(image)
    image = image_enhancer_brightness.enhance(factor)
    image = cvtColor(np.asarray(image), COLOR_RGB2BGR)
    return image

def add_salt_noise(img):
    # # 指定信噪比
    # SNR = 0.999
    # # 获取总共像素个数
    # size = img.size
    # noiseSize = int(size * (1 - SNR))
    # 5分之一的概率生成1到5个噪点
    if randint(1, 5) == 4:
        noiseSize = randint(1, 5)
        # 对这些点加噪声
        for k in range(0, noiseSize):
            # 随机获取 某个点
            xi = int(np.random.uniform(0, img.shape[1]))
            xj = int(np.random.uniform(0, img.shape[0]))
            # 增加噪声
            if img.ndim == 2 or img.ndim == 3:
                img[xj, xi] = randint(0, 255)
    return img


def get_cameras(cam_preset_num=4):
    cnt = 0
    cameras_list = []
    for device in range(0, cam_preset_num):
        stream = VideoCapture(device, CAP_DSHOW)
        grabbed = stream.grab()
        stream.release()
        if not grabbed:
            break
        cnt = cnt + 1
        cameras_list.append(device)
    LOG.info(f"该设备所连接的相机数量为：{cnt}")
    return cameras_list


def change_hand_label(newlabel):
    for root, dirs, _ in os.walk(IMAGE_DATA_PATH + "hands"):
        for dir in dirs:
            path = os.path.join(root, dir)
            if dir.endswith("labels"):
                for root1, dirs1, _ in os.walk(path):
                    for dir1 in dirs1:
                        path1 = os.path.join(root1, dir1)
                        files = os.listdir(path1)
                        for file in files:
                            if file.endswith("txt"):
                                txt_path = os.path.join(path1, file)

                                SaveList = []
                                # 读取文本内容到列表
                                with open(txt_path, "r", encoding='utf-8') as f:
                                    for line in f:
                                        line = line.strip('\n')  # 删除换行符
                                        line_new = str(newlabel) + ' ' + line.split(' ', 1)[1]
                                        SaveList.append(line_new)
                                    f.close()

                                with open(txt_path, 'w') as f:
                                    for line in SaveList:
                                        f.write(line + '\n')
                                    f.close()

def copyfile(source_path, target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)

    if os.path.exists(source_path):
        for root, dirs, files in os.walk(source_path):
            for file in files:
                src_file = os.path.join(root, file)
                copy(src_file, target_path)

    LOG.info('复制成功')


import pickle
import numpy as np
from scipy.interpolate import interp1d

class FunctionFitter:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = interp1d(self.x, self.y)


    def plot(self):
        import matplotlib.pyplot as plt
        xx = np.linspace(self.x.min(), self.x.max(), 1000)
        yy = self.f(xx)
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False
        plt.plot(self.x, self.y, 'o', label='误差')
        plt.plot(xx, yy, label='拟合的误差函数')
        plt.title('关闭该窗口继续操作')
        plt.legend()
        plt.show()

    def save(self, filename):
        with open(filename, 'wb') as file:
            pickle.dump(self.f, file)

    @classmethod
    def load(cls, filename):
        with open(filename, 'rb') as file:
            f = pickle.load(file)
        x = f.x
        y = f.y
        return cls(x, y)