# -*- coding: utf-8 -*-
import os
from random import choice, randint
from playsound import playsound
from math import cos, sin
import numpy as np
from cv2 import (
    VideoCapture,
    CAP_DSHOW,
    cvtColor,
    COLOR_BGR2RGB,
    COLOR_RGB2BGR,
    CAP_PROP_FRAME_WIDTH,
    CAP_PROP_FRAME_HEIGHT,
    CAP_PROP_FPS,
)
from PIL import ImageEnhance, Image
import yaml
from shutil import copy
from scipy.optimize import leastsq
from threading import Thread
from abc import abstractmethod

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


class YamlHandler:
    def __init__(self, filename):
        self.filename = filename

    def read_yaml(self):
        """读取yaml文件数据"""
        with open(self.filename, encoding="utf-8") as f:
            return yaml.load(f.read(), Loader=yaml.FullLoader)

    def write_yaml(self, data):
        """在yaml文件写入数据"""
        with open(self.filename, encoding="utf-8", mode="w") as f:
            return yaml.dump(data, stream=f, allow_unicode=True)


class CurveFitting(object):
    def __init__(self):
        self.param_init = [0, 1500]

    def fun(self, param, angle, length):  # 定义拟合函数形式
        len_calibration, angle_init = param
        return angle_init - (angle + length * len_calibration) * 7.28

    def error(self, param, angle, length, engine):  # 拟合残差
        return self.fun(param, angle, length) - engine

    def calc(self, angle_list, len_list, a):
        params = leastsq(
            self.error, self.param_init, args=(angle_list, len_list, a)
        )  # 进行拟合
        LOG.info(f"拟合结果: {params[0]}")
        return params[0]


def coordinate_mapping(
    pixel_list, physical_rows, physical_cols, pixel_rows, pixel_cols
):
    # LOG.debug(f"坐标映射: {physical_rows}, {physical_cols}, {pixel_rows}, {pixel_cols}")
    data = []
    for x, y, c in pixel_list:
        x = x * physical_rows / pixel_rows
        y = y * physical_cols / pixel_cols
        n = get_name_by_class(c)
        data.append([x, y, c, n])
    return data


def plane_coordinate_transform(
    coordinate_x, coordinate_y, transform_x, transform_y, transform_angle
):
    # 点旋转矩阵
    # 首先将点绕原点旋转 transform_angle 角度，transform_angle 为正值时，实现的是逆时针旋转
    # 然后将点在 x 方向平移 transform_x，在 y 方向平移 transform_y
    transform_matrix = [
        [cos(transform_angle), -sin(transform_angle), transform_x],
        [sin(transform_angle), cos(transform_angle), transform_y],
        [0, 0, 1],
    ]

    input_coordinate = np.array([coordinate_x, coordinate_y, 1])
    transform_matrix = np.array(transform_matrix)

    # x' = x * cos(θ) - y * sin(θ) + transform_x
    # y' = x * sin(θ) + y * cos(θ) + transform_y
    output_coordinate = np.dot(transform_matrix, input_coordinate.T)
    return output_coordinate[0], output_coordinate[1]


def plane_coordinate_transform2(
    arm, coordinate_x, coordinate_y, transform_x, transform_y
):
    """
    坐标系转换

    Returns:
        tuple: 变换后的 (x', y') 坐标
    """
    if arm == 'openarm':
        # 原始坐标系(x轴朝左、y轴朝上),变化后坐标系(x轴朝上、y轴朝左)
        # 实现 x、y 坐标轴对换后, 上移translate_y, 再右移translate_x
        # y - A (变化后x轴朝上，上移translate_x相当于减translate_x)
        new_x = coordinate_y - transform_y
        new_y = coordinate_x + transform_x  # x + B
    elif arm == 'uarm':
        # 原始坐标系(x轴朝左、y轴朝上),变化后坐标系(x轴朝上、y轴朝下)
        # 实现 x 轴不变、y 坐标轴变为反向后, 上移translate_y, 再右移translate_x
        new_x = coordinate_x + transform_x
        new_y = transform_y - coordinate_y
    return new_x, new_y


# 播放音频的模块会抛出异常,是由于windows不支持utf-16编码，需修改playsound源码
# 参考链接：https://blog.csdn.net/lj606/article/details/122354958
def play_sound_thread(*args):
    thread = Thread(target=play_sound, args=args)
    thread.start()


def play_sound(*args):
    SOUND_PATH = ROOT + "/sound/"
    for str_sound in args:
        path_list = []
        for filename in os.listdir(SOUND_PATH):
            if filename.startswith(str_sound):
                path_list.append(SOUND_PATH + filename)
        # todo: macos不支持
        # playsound(choice(path_list))


def get_name_by_class(_class):
    file_path = GlobalVar.get_value("DATA_YAML_PATH")
    if file_path is None:
        file_path = str(ROOT) + "../yolov5/data.yaml"  # 默认
    data = YamlHandler(file_path).read_yaml()
    name = data["names"][_class]
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
        stream = VideoCapture(device)  # , CAP_DSHOW)
        if stream.isOpened():
            # grabbed = stream.grab()
            grabbed, frame = stream.read()
            if grabbed:
                cnt = cnt + 1
                cameras_list.append(device)

                width = stream.get(CAP_PROP_FRAME_WIDTH)
                height = stream.get(CAP_PROP_FRAME_HEIGHT)
                fps = stream.get(CAP_PROP_FPS)
                LOG.info(f"摄像头 {device}: {width}x{height}, FPS: {fps}")
        stream.release()

    LOG.info(f"该设备所连接的相机数量为：{cnt}")
    return cameras_list


def change_hand_label(newlabel):
    for root, dirs, _ in os.walk(IMAGE_DATA_PATH + "hands/output_yolo"):
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
                                with open(txt_path, "r", encoding="utf-8") as f:
                                    for line in f:
                                        line = line.strip("\n")  # 删除换行符
                                        line_new = (
                                            str(newlabel) + " " + line.split(" ", 1)[1]
                                        )
                                        SaveList.append(line_new)
                                    f.close()

                                with open(txt_path, "w") as f:
                                    for line in SaveList:
                                        f.write(line + "\n")
                                    f.close()


def copyfile(source_path, target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)

    if os.path.exists(source_path):
        for root, dirs, files in os.walk(source_path):
            for file in files:
                src_file = os.path.join(root, file)
                copy(src_file, target_path)

    LOG.info("复制成功")


import pickle
import numpy as np
from scipy.interpolate import interp1d


class FunctionFitter:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = interp1d(self.x, self.y)  # 线性插值法纠偏

    def plot(self, title=""):
        import matplotlib.pyplot as plt

        xx = np.linspace(self.x.min(), self.x.max(), 1000)
        yy = self.f(xx)
        plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False
        plt.plot(self.x, self.y, "o", label="误差")
        plt.plot(xx, yy, label="拟合的误差函数")
        plt.title(f"{title} (关闭该窗口继续操作)")
        plt.legend()
        plt.show()

    def save(self, filename):
        with open(filename, "wb") as file:
            pickle.dump(self.f, file)

    @classmethod
    def load(cls, filename):
        with open(filename, "rb") as file:
            f = pickle.load(file)
        x = f.x
        y = f.y
        return cls(x, y)


import numpy as np
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class TrilinearCalibrator:
    """基于三线性插值的机械臂坐标误差校准类"""

    def __init__(self, grid_resolution=10):
        """
        初始化校准模型

        Parameters:
        -----------
        grid_resolution : int or tuple
            网格分辨率，每个维度的采样点数
        """
        if isinstance(grid_resolution, int):
            self.grid_resolution = (grid_resolution, grid_resolution, grid_resolution)
        else:
            self.grid_resolution = grid_resolution

        self.interpolator_x = None
        self.interpolator_y = None
        self.interpolator_z = None
        self.x_grid = None
        self.y_grid = None
        self.z_grid = None
        self.workspace_bounds = None

    def create_grid(self, target_coords):
        """
        根据测量数据创建插值网格

        Parameters:
        -----------
        target_coords : array-like, shape (n, 3)
            目标坐标数组

        Returns:
        --------
        grid_points : tuple
            (x_grid, y_grid, z_grid) 网格点坐标
        """
        # 确定工作空间边界（添加5%的边界余量）
        mins = np.min(target_coords, axis=0)
        maxs = np.max(target_coords, axis=0)
        ranges = maxs - mins
        ranges = np.maximum(ranges, 1.0)  # 确保至少1mm的范围
        margin = 0.05 * ranges

        self.workspace_bounds = {
            "x_min": mins[0] - margin[0],
            "x_max": maxs[0] + margin[0],
            "y_min": mins[1] - margin[1],
            "y_max": maxs[1] + margin[1],
            "z_min": mins[2] - margin[2],
            "z_max": maxs[2] + margin[2],
        }

        # 创建规则网格
        self.x_grid = np.linspace(
            self.workspace_bounds["x_min"],
            self.workspace_bounds["x_max"],
            self.grid_resolution[0],
        )
        self.y_grid = np.linspace(
            self.workspace_bounds["y_min"],
            self.workspace_bounds["y_max"],
            self.grid_resolution[1],
        )
        self.z_grid = np.linspace(
            self.workspace_bounds["z_min"],
            self.workspace_bounds["z_max"],
            self.grid_resolution[2],
        )
        # 新增：确保网格严格递增（避免数值误差导致的问题）
        self.x_grid = np.sort(self.x_grid)
        self.y_grid = np.sort(self.y_grid)
        self.z_grid = np.sort(self.z_grid)

        # 新增：去除重复值
        self.x_grid = np.unique(self.x_grid)
        self.y_grid = np.unique(self.y_grid)
        self.z_grid = np.unique(self.z_grid)
        return self.x_grid, self.y_grid, self.z_grid

    def fit(self, target_coords, actual_coords, method="rbf"):
        """
        将离散测量点的误差拟合到规则网格上

        Parameters:
        -----------
        target_coords : array-like, shape (n, 3)
            测量的目标坐标
        actual_coords : array-like, shape (n, 3)
            测量的实际坐标
        method : str
            拟合方法 'rbf' (径向基函数) 或 'idw' (反距离权重)

        Returns:
        --------
        error_grids : tuple
            (error_x_grid, error_y_grid, error_z_grid)
        """
        from scipy.interpolate import Rbf

        # 计算测量点的误差
        errors = actual_coords - target_coords

        # 创建网格点的坐标矩阵
        X, Y, Z = np.meshgrid(self.x_grid, self.y_grid, self.z_grid, indexing="ij")

        if method == "rbf":
            # 使用径向基函数插值
            rbf_x = Rbf(
                target_coords[:, 0],
                target_coords[:, 1],
                target_coords[:, 2],
                errors[:, 0],
                function="multiquadric",
                smooth=0.1,
            )
            rbf_y = Rbf(
                target_coords[:, 0],
                target_coords[:, 1],
                target_coords[:, 2],
                errors[:, 1],
                function="multiquadric",
                smooth=0.1,
            )
            rbf_z = Rbf(
                target_coords[:, 0],
                target_coords[:, 1],
                target_coords[:, 2],
                errors[:, 2],
                function="multiquadric",
                smooth=0.1,
            )

            error_x_grid = rbf_x(X, Y, Z)
            error_y_grid = rbf_y(X, Y, Z)
            error_z_grid = rbf_z(X, Y, Z)

        elif method == "idw":
            # 使用反距离权重插值
            error_x_grid = self._idw_interpolation(target_coords, errors[:, 0], X, Y, Z)
            error_y_grid = self._idw_interpolation(target_coords, errors[:, 1], X, Y, Z)
            error_z_grid = self._idw_interpolation(target_coords, errors[:, 2], X, Y, Z)

        return error_x_grid, error_y_grid, error_z_grid

    def _idw_interpolation(self, points, values, X, Y, Z, power=2):
        """反距离权重插值"""
        grid_shape = X.shape
        result = np.zeros(grid_shape)

        X_flat = X.flatten()
        Y_flat = Y.flatten()
        Z_flat = Z.flatten()

        for i in range(len(X_flat)):
            # 计算到所有测量点的距离
            distances = np.sqrt(
                (points[:, 0] - X_flat[i]) ** 2
                + (points[:, 1] - Y_flat[i]) ** 2
                + (points[:, 2] - Z_flat[i]) ** 2
            )

            # 避免除零
            distances = np.maximum(distances, 1e-10)

            # 反距离权重
            weights = 1.0 / (distances**power)
            weights /= np.sum(weights)

            # 加权平均
            result.flat[i] = np.sum(weights * values)

        return result.reshape(grid_shape)

    def train(self, target_coords, actual_coords, method="rbf"):
        """
        使用测量数据校准模型

        Parameters:
        -----------
        target_coords : array-like, shape (n, 3)
            目标坐标数组
        actual_coords : array-like, shape (n, 3)
            实际测量坐标数组
        method : str
            拟合方法 'rbf' 或 'idw'

        Returns:
        --------
        stats : dict
            校准结果统计
        """
        target_coords = np.array(target_coords)
        actual_coords = np.array(actual_coords)

        print("正在创建插值网格...")
        self.create_grid(target_coords)

        print(f"正在使用 {method} 方法拟合误差场...")
        error_x_grid, error_y_grid, error_z_grid = self.fit(
            target_coords, actual_coords, method=method
        )

        # 创建三线性插值器
        print("正在创建三线性插值器...")
        self.interpolator_x = RegularGridInterpolator(
            (self.x_grid, self.y_grid, self.z_grid),
            error_x_grid,
            method="linear",
            bounds_error=False,
            fill_value=None,  # 外推
        )
        self.interpolator_y = RegularGridInterpolator(
            (self.x_grid, self.y_grid, self.z_grid),
            error_y_grid,
            method="linear",
            bounds_error=False,
            fill_value=None,
        )
        self.interpolator_z = RegularGridInterpolator(
            (self.x_grid, self.y_grid, self.z_grid),
            error_z_grid,
            method="linear",
            bounds_error=False,
            fill_value=None,
        )

        # 计算校准精度
        self.target_coords = target_coords
        self.actual_coords = actual_coords
        self.predicted_errors = self.predict(target_coords)
        self.actual_errors = actual_coords - target_coords
        self.residual_errors = self.actual_errors - self.predicted_errors

        stats = {
            "rmse": np.sqrt(np.mean(self.residual_errors**2)),
            "max_error": np.max(np.abs(self.residual_errors)),
            "mean_error": np.mean(np.abs(self.residual_errors), axis=0),
            "std_error": np.std(self.residual_errors, axis=0),
            "grid_size": self.grid_resolution,
            "workspace_bounds": self.workspace_bounds,
        }

        print("校准完成！")
        return stats

    def predict(self, coords):
        """
        预测给定坐标的误差

        Parameters:
        -----------
        coords : array-like, shape (n, 3) or (3,)
            坐标点

        Returns:
        --------
        errors : array-like
            预测的误差 [dx, dy, dz]
        """
        if self.interpolator_x is None:
            raise ValueError("模型未校准，请先调用 calibrate() 方法")

        coords = np.atleast_2d(coords)

        error_x = self.interpolator_x(coords)
        error_y = self.interpolator_y(coords)
        error_z = self.interpolator_z(coords)

        errors = np.column_stack([error_x, error_y, error_z])
        return errors.squeeze()

    def calibrate(self, target_coords):
        """
        对目标坐标进行误差补偿

        Parameters:
        -----------
        target_coords : array-like, shape (n, 3) or (3,)
            目标坐标

        Returns:
        --------
        compensated_coords : array-like
            补偿后的坐标
        """
        target_coords = np.atleast_2d(target_coords)
        predicted_errors = self.predict(target_coords)

        # 补偿：目标坐标 + 预测误差 = 应发送坐标
        compensated_coords = target_coords + predicted_errors

        return compensated_coords.squeeze()

    def save_model(self, filename):
        """保存校准模型"""
        np.savez(
            filename,
            x_grid=self.x_grid,
            y_grid=self.y_grid,
            z_grid=self.z_grid,
            error_x=self.interpolator_x.values,
            error_y=self.interpolator_y.values,
            error_z=self.interpolator_z.values,
            workspace_bounds=self.workspace_bounds,
        )
        print(f"模型已保存到 {filename}")

    @classmethod
    def load_model(cls, filename):
        instance = cls(grid_resolution=10)
        """加载校准模型"""
        data = np.load(filename, allow_pickle=True)
        instance.x_grid = data["x_grid"]
        instance.y_grid = data["y_grid"]
        instance.z_grid = data["z_grid"]
        instance.workspace_bounds = data["workspace_bounds"].item()

        instance.interpolator_x = RegularGridInterpolator(
            (instance.x_grid, instance.y_grid, instance.z_grid),
            data["error_x"],
            method="linear",
            bounds_error=False,
            fill_value=None,
        )
        instance.interpolator_y = RegularGridInterpolator(
            (instance.x_grid, instance.y_grid, instance.z_grid),
            data["error_y"],
            method="linear",
            bounds_error=False,
            fill_value=None,
        )
        instance.interpolator_z = RegularGridInterpolator(
            (instance.x_grid, instance.y_grid, instance.z_grid),
            data["error_z"],
            method="linear",
            bounds_error=False,
            fill_value=None,
        )
        print(f"模型已从 {filename} 加载")
        return instance

    def plot(self):
        """
        可视化校准结果
        """
        # self.target_coords = target_coords
        # self.actual_coords = actual_coords
        # self.predicted_errors = self.predict(target_coords)
        # self.actual_errors = actual_coords - target_coords
        # self.residual_errors = self.actual_errors - self.predicted_errors

        # 可视化
        fig = plt.figure(figsize=(18, 5))

        # 原始误差分布
        ax1 = fig.add_subplot(131, projection="3d")
        original_errors = self.actual_errors
        error_norms = np.linalg.norm(original_errors, axis=1)
        scatter1 = ax1.scatter(
            self.target_coords[:, 0],
            self.target_coords[:, 1],
            self.target_coords[:, 2],
            c=error_norms,
            cmap="Reds",
            s=30,
        )
        ax1.grid(True, linestyle="--", alpha=0.7, color="gray")  # 开启网格，设置样式
        _ = [
            ax1.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
            for (x, y, z), e in zip(self.target_coords, error_norms)
        ]  # 为每个点添加误差标签

        ax1.set_xlabel("X (mm)")
        ax1.set_ylabel("Y (mm)")
        ax1.set_zlabel("Z (mm)")
        ax1.set_title("原始误差分布")
        plt.colorbar(scatter1, ax=ax1, label="误差大小 (mm)", shrink=0.6)

        # 插值预测误差
        ax2 = fig.add_subplot(132, projection="3d")
        predicted_errors = self.predict(self.target_coords)
        error_norms = np.linalg.norm(predicted_errors, axis=1)
        scatter2 = ax2.scatter(
            self.target_coords[:, 0],
            self.target_coords[:, 1],
            self.target_coords[:, 2],
            c=error_norms,
            cmap="Blues",
            s=30,
        )
        ax2.set_xlabel("X (mm)")
        ax2.set_ylabel("Y (mm)")
        ax2.set_zlabel("Z (mm)")
        ax2.set_title("插值预测误差")

        _ = [
            ax2.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
            for (x, y, z), e in zip(self.target_coords, error_norms)
        ]  # 为每个点添加误差标签
        plt.colorbar(scatter2, ax=ax2, label="误差大小 (mm)", shrink=0.6)

        # 校准后残余误差
        ax3 = fig.add_subplot(133, projection="3d")
        residual_errors = original_errors - predicted_errors
        error_norms = np.linalg.norm(residual_errors, axis=1)
        scatter3 = ax3.scatter(
            self.target_coords[:, 0],
            self.target_coords[:, 1],
            self.target_coords[:, 2],
            c=error_norms,
            cmap="Greens",
            s=30,
        )
        ax3.set_xlabel("X (mm)")
        ax3.set_ylabel("Y (mm)")
        ax3.set_zlabel("Z (mm)")
        ax3.set_title("校准后残余误差")

        _ = [
            ax3.text(x, y, z, f"{x:.0f},{y:.0f},{z:.0f},{e:.2f}mm", fontsize=8)
            for (x, y, z), e in zip(self.target_coords, error_norms)
        ]  # 为每个点添加误差标签
        plt.colorbar(scatter3, ax=ax3, label="误差大小 (mm)", shrink=0.6)

        plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False
        plt.tight_layout()
        plt.show()
