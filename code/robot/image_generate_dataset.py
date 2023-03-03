# -*- coding: utf-8 -*-
import cv2
import numpy as np
import random
import time
import os
import Augmentor
import shutil
from tkinter import messagebox

from robot.tools import transparence_to_white,random_brightness,add_salt_noise, YamlHandler, copyfile, change_hand_label
from robot import ROOT, PARAMS_YAML, IMAGE_DATA_PATH, LOG

DATA_NAME = YamlHandler(PARAMS_YAML).read_yaml()['data_name']

BACKGROUND_INPUT_PATH = IMAGE_DATA_PATH + DATA_NAME + "/background_imgs/"
FOREGROUND_INPUT_PATH = IMAGE_DATA_PATH + DATA_NAME + "/foreground_imgs/"

IMAGES_OUT_PATH_VAL = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/images/val/"
LABELS_OUT_PATH_VAL = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/labels/val/"

IMAGES_OUT_PATH_TEST = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/images/test/"
LABELS_OUT_PATH_TEST = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/labels/test/"

IMAGES_OUT_PATH_TRAIN = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/images/train/"
LABELS_OUT_PATH_TRAIN = IMAGE_DATA_PATH + DATA_NAME + "/output_yolo/labels/train/"

IMAGES_HAND_TEST = IMAGE_DATA_PATH + "hands/images/test/"
LABELS_HAND_TEST = IMAGE_DATA_PATH + "hands/labels/test/"

IMAGES_HAND_TRAIN = IMAGE_DATA_PATH + "hands/images/train/"
LABELS_HAND_TRAIN = IMAGE_DATA_PATH + "hands/labels/train/"

IMAGES_HAND_VAL = IMAGE_DATA_PATH + "hands/images/val/"
LABELS_HAND_VAL = IMAGE_DATA_PATH + "hands/labels/val/"

PER_BACKGROUND_NUM = 188
PER_SAMPLE_NUM = 20


class YoloDataProducer(object):
    def __init__(self, background_file=BACKGROUND_INPUT_PATH,
                 foreground_file=FOREGROUND_INPUT_PATH, per_num=1, overlap_factor=0.8, max_num=5, is_circle=True,
                 root=None, progressbar=None):
        """
        :param background_file: 背景文件夹名
        :param foreground_file: 前景文件夹名
        :param per_num: 每张背景需要生成的数据量
        :param overlap_factor: 前景间重叠因子，越小随机的各个前景间可重合度越高
        :param max_num: 每个前景在背景中的随机的最大个数
        :param is_circle: 前景是否是圆形
        """
        self.overlap_factor = overlap_factor
        self.background_file = background_file
        self.foreground_file = foreground_file
        self.per_num = per_num
        self.max_num = max_num
        self.is_circle = is_circle

        # GUI显示进度条
        self.root = root
        self.progressbar = progressbar

        self.background_list = []

        self.foreground_list_map = {}  # key:种类 value:图像文件名list
        self.class_map = {}  # key:种类 value:种类名

        self.num_to_file_map = {1: [IMAGES_OUT_PATH_TRAIN, LABELS_OUT_PATH_TRAIN],
                                2: [IMAGES_OUT_PATH_TRAIN, LABELS_OUT_PATH_TRAIN],
                                3: [IMAGES_OUT_PATH_TRAIN, LABELS_OUT_PATH_TRAIN],
                                4: [IMAGES_OUT_PATH_TEST, LABELS_OUT_PATH_TEST],
                                5: [IMAGES_OUT_PATH_VAL, LABELS_OUT_PATH_VAL], }

    # 数据增广
    def augmente(self):
        p = Augmentor.Pipeline(BACKGROUND_INPUT_PATH)
        p.rotate(probability=0.5, max_left_rotation=10, max_right_rotation=10)
        p.shear(probability=0.25, max_shear_left=5, max_shear_right=5)
        p.random_brightness(probability=0.9, min_factor=0.7, max_factor=1.3)
        p.random_color(probability=0.9, min_factor=0.7, max_factor=1.3)
        p.random_contrast(probability=0.9, min_factor=0.7, max_factor=1.3)
        p.zoom(probability=0.8, min_factor=0.7, max_factor=1)
        p.scale(probability=0.2, scale_factor=1.1)
        p.scale(probability=0.2, scale_factor=1.2)
        p.scale(probability=0.2, scale_factor=1.3)
        p.rotate_random_90(probability=0.75)
        p.sample(int(len(os.listdir(BACKGROUND_INPUT_PATH)) * PER_BACKGROUND_NUM))

        show_progress(self.root, self.progressbar, 5)

        p = Augmentor.Pipeline(FOREGROUND_INPUT_PATH)
        if self.is_circle:
            p.rotate_without_crop(probability=0.6, max_left_rotation=45, max_right_rotation=45)
        p.shear(probability=0.3, max_shear_left=3, max_shear_right=3)
        p.random_color(probability=0.5, min_factor=0.7, max_factor=1.3)
        p.random_contrast(probability=0.5, min_factor=0.3, max_factor=1.3)
        p.rotate_random_90(probability=0.75)
        p.zoom(probability=0.8, min_factor=0.7, max_factor=1)
        p.scale(probability=0.2, scale_factor=1.1)
        p.scale(probability=0.2, scale_factor=1.2)
        p.scale(probability=0.2, scale_factor=1.3)
        p.sample(int(len(os.listdir(FOREGROUND_INPUT_PATH)) * PER_SAMPLE_NUM))

        show_progress(self.root, self.progressbar, 5)

        for filename in os.listdir(self.background_file + "output"):
            self.background_list.append(os.path.join(self.background_file + "output", filename))
        for filename in os.listdir(self.foreground_file + "output"):
            temp_name = os.path.join(self.foreground_file + "output", filename)
            self.record_class_map(temp_name)

        i = 0
        sorted_map = dict(sorted(self.class_map.items(), key=lambda x: x[0]))
        print(sorted_map)
        for key in sorted_map.keys():
            if key != i:
                print(sorted_map, key, i)
                messagebox.showerror('错误', f'数据集的序号没有按照从0开始的严格自增的规则，请修改序号为 {key} 的图片',
                                     self.root)
                return False
            i += 1
        return True

    def transparence_to_white_all(self):
        for filename in os.listdir(FOREGROUND_INPUT_PATH + "output"):
            if not filename.endswith(".png"):
                continue
            temp_name = os.path.join(FOREGROUND_INPUT_PATH + "output", filename)
            # out_name = os.path.join(FOREGROUND_OUT_PATH, filename)
            img_foreground = cv2.imread(temp_name, cv2.IMREAD_UNCHANGED)
            img_foreground = transparence_to_white(img_foreground)
            cv2.imwrite(temp_name, img_foreground)

    def delete(self, ):

        if os.path.exists(IMAGES_OUT_PATH_VAL):
            shutil.rmtree(IMAGES_OUT_PATH_VAL)
        if os.path.exists(LABELS_OUT_PATH_VAL):
            shutil.rmtree(LABELS_OUT_PATH_VAL)
        show_progress(self.root, self.progressbar, 2.5)

        if os.path.exists(IMAGES_OUT_PATH_TEST):
            shutil.rmtree(IMAGES_OUT_PATH_TEST)
        if os.path.exists(LABELS_OUT_PATH_TEST):
            shutil.rmtree(LABELS_OUT_PATH_TEST)
        show_progress(self.root, self.progressbar, 2.5)

        if os.path.exists(IMAGES_OUT_PATH_TRAIN):
            shutil.rmtree(IMAGES_OUT_PATH_TRAIN)
        if os.path.exists(LABELS_OUT_PATH_TRAIN):
            shutil.rmtree(LABELS_OUT_PATH_TRAIN)
        show_progress(self.root, self.progressbar, 2.5)

        if os.path.exists(BACKGROUND_INPUT_PATH + "output"):
            shutil.rmtree(BACKGROUND_INPUT_PATH + "output")
        if os.path.exists(FOREGROUND_INPUT_PATH + "output"):
            shutil.rmtree(FOREGROUND_INPUT_PATH + "output")
        show_progress(self.root, self.progressbar, 2.5)

        os.makedirs(IMAGES_OUT_PATH_VAL)
        os.makedirs(LABELS_OUT_PATH_VAL)
        os.makedirs(IMAGES_OUT_PATH_TEST)
        os.makedirs(LABELS_OUT_PATH_TEST)
        os.makedirs(IMAGES_OUT_PATH_TRAIN)
        os.makedirs(LABELS_OUT_PATH_TRAIN)

    # 生成数据集
    def produce(self):
        if len(self.foreground_list_map) == 0:
            LOG.error("无数据可生成数据集")
            return
        random.shuffle(self.background_list)
        for background_name in self.background_list:
            img_background = cv2.imread(background_name)
            for i in range(self.per_num):
                img_background_new = img_background.copy()
                self.produce_single(img_background_new)
            LOG.debug(f"已生成背景为  {background_name}  的数据")
            show_progress(self.root, self.progressbar, 70 / len(self.background_list))

    # 写yolo的yaml文件
    def write_yolo_yaml(self):
        if len(self.class_map) > 0:
            file_path = ROOT + "../yolov5/data/self_data.yaml"  # code
            # file_path = ROOT + "yolov5/data/self_data.yaml"  # exe
            data = YamlHandler(file_path).read_yaml()
            LOG.debug(f"yaml修改前数据：{data}")
            # 将data数据写入yaml
            sorted_map = dict(sorted(self.class_map.items(), key=lambda x: x[0]))
            data['nc'] = len(sorted_map)
            name_values = sorted_map.values()
            data['names'] = list(name_values)
            data['path'] = IMAGE_DATA_PATH.replace(ROOT, '', 1) + DATA_NAME + '/output_yolo'
            YamlHandler(file_path).write_yaml(data)
            LOG.debug(f"yaml修改后数据：{data}")

    def produce_single(self, img_background):
        random_point_list = []  # 放置点信息
        start_index = 0
        order = 0
        foreground_list = []
        # 从各种类中随机一部分图像出来
        for _class in self.foreground_list_map.keys():
            for j in range(random.randint(1, self.max_num)):
                random_index = random.randint(0, len(self.foreground_list_map[_class]) - 1)
                foreground_list.append(self.foreground_list_map[_class][random_index])
        random.shuffle(foreground_list)
        foreground_list = foreground_list[0:random.randint(1, len(foreground_list))]

        for filename in foreground_list:
            img_foreground = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
            try:
                img_foreground = transparence_to_white(img_foreground)
                img_foreground = add_salt_noise(img_foreground)
            except:
                pass
            # 将前景转化为二值化图mask
            img_gray = cv2.cvtColor(img_foreground, cv2.COLOR_BGR2GRAY)
            # img_gray = np.max(img_foreground, axis=2)
            ret, mask = cv2.threshold(img_gray, 254, 255, cv2.THRESH_BINARY)

            # 对mask取反操作颠倒黑白
            mask_inv = cv2.bitwise_not(mask)

            # 对前景进行掩膜操作，去除前景中黑的部分
            img_fg = cv2.bitwise_and(img_foreground, img_foreground, mask=mask_inv)

            # 随机前景在背景中的放置点
            order += 1
            self.random_point(img_background, img_foreground, filename, random_point_list)

            # 根据放置点将前景放置进背景中
            for x, y, rows, cols, _, _, _ in random_point_list[start_index:]:
                # 截取背景中的放置区域
                roi = img_background[x:rows + x, y:cols + y]

                # 对放置区域进行掩膜操作，将前景中黑的部分换为背景的像素
                roi_bg = cv2.bitwise_and(roi, roi, mask=mask)

                # 混合背景图和前景图，得到混合图dst
                dst = cv2.add(roi_bg, img_fg)

                # 将混合图dst覆盖背景的放置区域，得到最终的效果图
                img_background[x:rows + x, y:cols + y] = dst
            start_index = len(random_point_list)

        # 生成yolo格式的数据集
        save_name = time.time().__str__()

        # 训练集:测试集:验证集 = 6:2:2
        random_file_num = random.randint(1, 5)
        img_background = random_brightness(img_background)
        cv2.imwrite(self.num_to_file_map[random_file_num][0] + save_name + ".png", img_background)
        self.to_yolo(self.num_to_file_map[random_file_num][1] + save_name + ".txt", random_point_list)

    def random_point(self, img_background, img_foreground, file_name, point_list):
        # random.seed(time.time())
        rows_b, cols_b, _ = img_background.shape
        rows_f, cols_f, _ = img_foreground.shape
        rows_random, cols_random = rows_b - rows_f, cols_b - cols_f  # 可随机的范围
        # 前景不可超出背景
        if rows_random <= 0 or cols_random <= 0:
            return

        is_success = False  # 是否成功随机到点
        count_random = 0  # while循环的总次数
        while not is_success:
            x = random.randrange(rows_random)
            y = random.randrange(cols_random)
            count_random += 1

            b_has = False
            for x1, y1, rows1, cols1, file_name1, _, _ in point_list:
                if x1 < x and y1 < y:
                    if (x - x1) < (rows1 * self.overlap_factor) and (y - y1) < (cols1 * self.overlap_factor):
                        b_has = True
                        break
                elif x1 > x and y1 < y:
                    if (x1 - x) < (rows_f * self.overlap_factor) and (y - y1) < (cols1 * self.overlap_factor):
                        b_has = True
                        break
                elif x1 < x and y1 > y:
                    if (x - x1) < (rows1 * self.overlap_factor) and (y1 - y) < (cols_f * self.overlap_factor):
                        b_has = True
                        break
                else:
                    if (x1 - x) < (rows_f * self.overlap_factor) and (y1 - y) < (cols_f * self.overlap_factor):
                        b_has = True
                        break

            if not b_has:
                point_list.append((x, y, rows_f, cols_f, file_name, rows_b, cols_b))
                is_success = True

            # 如果循环次数大于30，就退出循环,避免陷入死循环
            if count_random > 30:
                break
        return point_list

    def to_yolo(self, file_name, random_point_list):
        data = []
        for x, y, rows_f, cols_f, filename, rows_b, cols_b in random_point_list:
            _class = self.get_class_from_name(filename)
            data.append(
                [_class, (y + cols_f / 2) / cols_b, (x + rows_f / 2) / rows_b, cols_f / cols_b, rows_f / rows_b])
            np.savetxt(file_name, data, '%s')

    def get_class_from_name(self, filename):
        _list = filename.split('-')
        _class = 0
        if len(_list) > 1:
            _class = int(_list[1])
        return _class

    def record_class_map(self, filename):
        _list = filename.split('-')
        if len(_list) > 1:
            _class = int(_list[1])
            self.class_map[_class] = _list[0].split('_')[-1]
            try:
                self.foreground_list_map[_class].append(filename)
            except:
                _temp_list = []
                _temp_list.append(filename)
                self.foreground_list_map[_class] = _temp_list

    def add_hand_data(self):
        hand_class = len(self.class_map)
        LOG.debug(f'手的种类是：{hand_class}')
        self.class_map[hand_class] = 'hand'
        change_hand_label(hand_class)

        copyfile(IMAGES_HAND_TEST, IMAGES_OUT_PATH_TEST)
        copyfile(LABELS_HAND_TEST, LABELS_OUT_PATH_TEST)

        copyfile(IMAGES_HAND_TRAIN, IMAGES_OUT_PATH_TRAIN)
        copyfile(LABELS_HAND_TRAIN, LABELS_OUT_PATH_TRAIN)

        copyfile(IMAGES_HAND_VAL, IMAGES_OUT_PATH_VAL)
        copyfile(LABELS_HAND_VAL, LABELS_OUT_PATH_VAL)

        show_progress(self.root, self.progressbar, 10)


def show_progress(root, progressbar, add_value):
    if root is not None and progressbar is not None:
        progressbar['value'] += add_value
        root.update()


def run(per_num, overlap_factor, max_num, is_circle, root=None, progressbar=None):
    producer = YoloDataProducer(per_num=per_num, overlap_factor=overlap_factor, max_num=max_num, is_circle=is_circle,
                                root=root, progressbar=progressbar)
    producer.delete()
    if producer.augmente():
        producer.produce()
        producer.add_hand_data()
        producer.write_yolo_yaml()
        messagebox.showinfo('提示', '已生成数据集', parent=root)


if __name__ == '__main__':
    producer = YoloDataProducer(per_num=2)
    producer.produce()
