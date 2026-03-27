# -*- coding: utf-8 -*-
import time

import cv2
import os
import tkinter
from tkinter import ttk, filedialog
from PIL import Image, ImageTk
import numpy as np
import threading
import json


from yolov5.detect_self import YoloDetecter
from robot import robot_master, LOG, ROOT
from robot.robot_master import Observable
from robot.tools import get_cameras, YamlHandler, GlobalVar
from image.image_find_focus import FocusFinder
from llm.multimodal_recognition_ark import MultimodalRecognizerArk

DEFAULT_MODEL_PATH = ROOT + "../yolov5/runs/train/exp9/weights/best.pt"


def yolo_to_pixel(yolo_list, rows_b, cols_b):
    data = []
    for x, y, w, h, c in yolo_list:
        pixel_y = y * cols_b
        pixel_x = x * rows_b
        data.append([pixel_x, pixel_y, c])
    return data


# 显示采集到的图像图片
def tk_show_img(panel, img):
    # 在窗口中显示这一帧图像
    cv2.imshow("Camera Frame", img)
    cv2.waitKey(
        1
    )  # cv2.waitKey(1) 是必要的，它会等待1毫秒，以便OpenCV能够处理窗口事件，如按键事件。如果没有这行代码，窗口可能会冻结，无法响应按键操作。

    # img = cv2.pyrDown(img)
    if img is not None:
        cv2image = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # 转换颜色从BGR到RGBA
        current_image = Image.fromarray(cv2image)  # 将图像转换成Image对象
        imgtk = ImageTk.PhotoImage(image=current_image)
        panel.pyimage1 = imgtk
        panel.config(image=imgtk)
        panel.update()


def tk_show_img_opencv_only(panel, img, suffix=""):
    if img is not None:
        # 只使用OpenCV显示，跳过Tkinter部分
        window_name = f"Camera Frame {suffix}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 创建可调整大小的窗口
        cv2.resizeWindow(window_name, 640, 640)  # 设置窗口大小为640x480
        # img = cv2.resize(img.copy(), (640, 640))
        cv2.imshow(window_name, img)

        # 在panel上显示状态信息而不是图像
        try:
            panel.config(
                text=f"图像显示在OpenCV窗口中\n按'q'退出\n图像尺寸: {img.shape}"
            )
            panel.update()
        except:
            pass

    # 如果按下'q'键就退出
    # if key == ord("q"):
    #     cv2.destroyAllWindows()
    #     return False
    return True


# 通过Yolo 视觉检测能力，判断用户行为，并根据用户行为，调用机械臂动作
class DetecterWindow(Observable):
    def __init__(self, root, window_flag_bit=None):
        super().__init__()
        self.detect_flag = False
        self.connect_flag = False
        self.self_yolo = None
        self.capture = None
        self.chess_think_depth = 3

        self.window_flag_bit = window_flag_bit
        self.robot_master = None

        self.window(root)

    def window(self, root):
        self.root = root
        self.root.config(width=400, height=1000)
        self.root.title("视觉检测")
        self.panel = tkinter.Label(self.root)
        self.panel.grid(row=0, column=1, sticky=("e", "w"))
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # 添加模型类型选择
        self.model_type = tkinter.StringVar(self.root)
        self.model_type.set("yolo")  # 默认选择YOLO

        self.model_type_label = tkinter.Label(self.root, text="模型类型:")
        self.model_type_label.grid(row=0, column=0)

        self.yolo_radio = tkinter.Radiobutton(
            self.root,
            text="YOLO",
            variable=self.model_type,
            value="yolo",
            command=self.toggle_model_config,
        )
        self.yolo_radio.grid(row=0, column=1)

        self.llm_radio = tkinter.Radiobutton(
            self.root,
            text="LLM",
            variable=self.model_type,
            value="llm",
            command=self.toggle_model_config,
        )
        self.llm_radio.grid(row=0, column=2)

        # LLM配置
        self.api_key = tkinter.StringVar(self.root)
        self.api_endpoint = tkinter.StringVar(self.root)

        self.api_key_label = tkinter.Label(self.root, text="API密钥:")
        self.api_key_entry = tkinter.Entry(
            self.root, textvariable=self.api_key, show="*"
        )

        self.api_endpoint_label = tkinter.Label(self.root, text="API接口地址:")
        self.api_endpoint_entry = tkinter.Entry(
            self.root, textvariable=self.api_endpoint
        )

        # YOLO相关配置
        self.path = tkinter.StringVar(self.root)
        self.path.set(os.path.abspath(DEFAULT_MODEL_PATH))

        self.path_label = tkinter.Label(self.root, text="模型路径:")
        self.path_label.grid(row=1, column=0)
        self.path_entry = tkinter.Entry(
            self.root, textvariable=self.path, state="readonly"
        )
        self.path_entry.grid(row=1, column=1, ipadx=200)

        self.patth_button = tkinter.Button(
            self.root, text="路径选择", command=self.select_path
        )
        self.patth_button.grid(row=1, column=2)

        self.label1 = tkinter.Label(
            self.root, text="请输入检测设备(GPU输入0,Apple输入mps,CPU输入cpu):"
        )
        self.inp1 = tkinter.Entry(self.root)
        self.inp1.insert(0, "cpu")
        self.label1.grid(row=2, column=0)
        self.inp1.grid(row=2, column=1)

        self.model_button = tkinter.Button(
            self.root, text="模型确认", command=self.load_model
        )
        self.model_button.grid(row=3, column=1)

        self.camera_label = tkinter.Label(self.root, text="请选择相机")
        self.cameraselect = ttk.Combobox(self.root)
        self.cameraselect.bind("<<ComboboxSelected>>", self.select_camera)
        self.cameraselect["value"] = get_cameras()

        # 开始视觉检测，识别用户行为
        self.detect_button = tkinter.Button(
            self.root, text="开始检测", command=self.start_detect_cmd
        )
        self.chess_think_depth_label = tkinter.Label(
            self.root,
            bg="#9FB6CD",
            width=80,
            text="象棋AI思考步数，步数越多，难度越高，超过5步会等较长时间。当前思考步数：3",
        )
        self.chess_think_depth_scale = tkinter.Scale(
            self.root,
            # label='象棋AI思考步数，步数越多，难度越高',
            from_=1,
            to=20,
            orient=tkinter.HORIZONTAL,  # 设置Scale控件平方向显示
            length=400,
            tickinterval=2,  # 设置刻度滑动条的间隔
            command=self.set_chess_think_depth,
        )  # 调用执行函数，是数值显示在 Label控件中
        self.connect_button = tkinter.Button(
            self.root, text="连接控制器", command=self.connect_cmd
        )
        self.connect_label = tkinter.Label(
            self.root,
            bg="#9FB6CD",
            width=80,
            text="点击「连接机械臂」按钮，开始下棋或物体分拣",
        )

        # 工作类型 0：未选择 1：五子棋 2：象棋 3：物体分类
        self.game_mode = tkinter.IntVar(self.root)

    def toggle_model_config(self):
        # 根据选择的模型类型显示相应的配置界面
        if self.model_type.get() == "yolo":
            # 显示YOLO配置，隐藏LLM配置
            self.path_label.grid(row=1, column=0)
            self.path_entry.grid(row=1, column=1, ipadx=200)
            self.patth_button.grid(row=1, column=2)
            self.label1.grid(row=2, column=0)
            self.inp1.grid(row=2, column=1)

            # 隐藏LLM配置
            self.api_key_label.grid_forget()
            self.api_key_entry.grid_forget()
            self.api_endpoint_label.grid_forget()
            self.api_endpoint_entry.grid_forget()

        elif self.model_type.get() == "llm":
            # 隐藏YOLO配置
            self.path_label.grid_forget()
            self.path_entry.grid_forget()
            self.patth_button.grid_forget()
            self.label1.grid_forget()
            self.inp1.grid_forget()

            # 显示LLM配置
            self.api_key_label.grid(row=1, column=0)
            self.api_key_entry.grid(row=1, column=1, ipadx=200)
            self.api_endpoint_label.grid(row=2, column=0)
            self.api_endpoint_entry.grid(row=2, column=1, ipadx=200)

    def select_camera(self, *args):
        self.camera_label.grid_forget()
        self.cameraselect.grid_forget()
        self.detect_button.grid(row=3, column=1)

        dev = int(self.cameraselect.get())
        self.open_camera(dev)

    def open_camera(self, dev):
        self.detect_flag = True
        self.capture = cv2.VideoCapture(dev)  # , cv2.CAP_DSHOW)
        img = self.get_video_frame()
        # tk_show_img_opencv_only(self.panel, img, suffix="[origin image]")
        self.panel.config(
            text=f"图像显示在OpenCV窗口中\n按'q'退出\n图像尺寸: {img.shape}"
        )
        self.panel.update()

        # 设置分辨率
        # self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def get_video_frame(self):
        # 摄像头读取,ret为是否成功打开摄像头,true,false。 frame为视频的每一帧图像
        retry_count = 0
        max_retries = 3
        while retry_count < max_retries:
            ret, frame = self.capture.read()
            if ret:
                return frame
            else:
                retry_count += 1
                LOG.debug(f"摄像头无数据，正在重试... ({retry_count}/{max_retries})")
                time.sleep(0.1)  # 等待100ms后重试

        # 重试3次后仍然失败
        # tkinter.messagebox.showerror(
        #     "错误", "摄像头无数据，已重试3次", parent=self.root
        # )
        return None

    def close(self):
        self.detect_flag = False
        self.root.destroy()
        if self.window_flag_bit is not None:
            self.window_flag_bit.value = self.window_flag_bit.value ^ (1 << 2)
        # 关闭所有OpenCV imgShow窗口
        cv2.destroyAllWindows()

    def connect_cmd(self):
        if self.connect_button["text"] == "连接控制器":
            if self.game_mode.get() == 0:
                tkinter.messagebox.showerror("错误", "未选择模式", parent=self.root)
                return

            if self.connect_robot(self.game_mode.get()):
                self.connect_button["text"] = "断开控制器"
            else:
                tkinter.messagebox.showerror(
                    "错误", "控制器未开启指令监听", parent=self.root
                )
        else:
            self.disconnect_robot()
            self.connect_button["text"] = "连接控制器"

    def connect_robot(self, game_mode):
        if self.connect_flag:
            return False
        if game_mode == 1:
            self.robot_master = robot_master.GobangRobotMaster()  # 五子棋
        elif game_mode == 2:
            self.robot_master = robot_master.ChessRobotMaster(  # 象棋
                think_depth=self.chess_think_depth
            )
        elif game_mode == 3:
            self.robot_master = robot_master.GrabRobotMaster()  # 物体分类

        if self.robot_master.connect_robot() == 0:  # 连接成功
            self.connect_flag = True

        self.chess_think_depth_scale.grid_forget()
        self.chess_think_depth_label.grid_forget()
        self.radio_button_gobang.grid_forget()
        self.radio_button_chess.grid_forget()
        self.radio_button_grab.grid_forget()

        # 注册观察者，消费视觉识别结果
        self.register(self.robot_master, "yolo_res")
        self.register(self.robot_master, "llm_res")
        self.register(self.robot_master, "safety")
        return True
        # else:
        # return False

    def disconnect_robot(self):
        self.unregister(self.robot_master, "yolo_res")
        self.unregister(self.robot_master, "llm_res")
        self.unregister(self.robot_master, "safety")
        self.connect_flag = False
        self.robot_master.close()
        self.chess_think_depth_scale.grid(row=3, column=1)
        self.chess_think_depth_label.grid(row=4, column=1)
        self.radio_button_gobang.grid(row=5, column=0)
        self.radio_button_chess.grid(row=5, column=1)
        self.radio_button_grab.grid(row=5, column=2)

        # 恢复模型选择配置显示
        self.toggle_model_config()

    def start_detect_cmd(self):
        self.detect_button.grid_forget()
        self.chess_think_depth_scale.grid(row=3, column=1)
        self.chess_think_depth_label.grid(row=4, column=1)

        self.radio_button_gobang = tkinter.Radiobutton(
            self.root, text="五子棋", variable=self.game_mode, value=1
        )
        self.radio_button_gobang.grid(row=5, column=0)

        self.radio_button_chess = tkinter.Radiobutton(
            self.root, text="象棋", variable=self.game_mode, value=2
        )
        self.radio_button_chess.grid(row=5, column=1)

        self.radio_button_grab = tkinter.Radiobutton(
            self.root, text="物体分类", variable=self.game_mode, value=3
        )
        self.radio_button_grab.grid(row=5, column=2)

        self.connect_button.grid(row=7, column=1)
        self.connect_label.grid(row=6, column=1)

        self.detect()

    def load_model(self):
        if self.model_type.get() == "yolo":
            dir = self.path.get()
            if not dir.endswith(".pt"):
                tkinter.messagebox.showerror("错误", "模型错误", parent=self.root)
            else:
                if self.inp1.get() == "cpu":
                    device = "cpu"
                elif self.inp1.get() == "mps":
                    import torch

                    if torch.backends.mps.is_available() == False:
                        tkinter.messagebox.showerror(
                            "错误", "当前系统版本不支持 mps,", parent=self.root
                        )
                    device = "mps"
                else:
                    from torch.cuda import is_available

                    if is_available() == False:
                        tkinter.messagebox.showerror(
                            "错误",
                            "cuda未安装或版本出错，不可使用GPU",
                            parent=self.root,
                        )
                        return
                    device = 0
                self.data_path = os.path.dirname(self.path.get()) + "/../data.yaml"
                self.self_yolo = YoloDetecter(
                    weights=dir, data=self.data_path, device=device
                )

                self.path_label.grid_forget()
                self.path_entry.grid_forget()
                self.patth_button.grid_forget()
                self.model_button.grid_forget()
                self.label1.grid_forget()
                self.inp1.grid_forget()

                # os.path.dirname(self.path.get()) + "/../data.yaml"
                GlobalVar.set_value(
                    "DATA_YAML_PATH", os.path.dirname(self.path.get()) + "/../data.yaml"
                )  # 该模型对应的数据集yaml文件
        else:
            # 处理LLM模型加载逻辑
            api_key = self.api_key.get()
            api_endpoint = self.api_endpoint.get()

            if not api_key:
                tkinter.messagebox.showerror("错误", "请输入API密钥", parent=self.root)
                return

            if not api_endpoint:
                tkinter.messagebox.showerror(
                    "错误", "请输入API接口地址", parent=self.root
                )
                return

            # 这里可以添加实际的LLM模型初始化代码
            self.llm_recognizer = MultimodalRecognizerArk(
                api_key=api_key, api_endpoint=api_endpoint
            )
            print(
                f"LLM配置已设置 - API密钥: {api_key[:4]}****, API端点: {api_endpoint}"
            )

            # 隐藏配置项，显示摄像头选择
            self.api_key_label.grid_forget()
            self.api_key_entry.grid_forget()
            self.api_endpoint_label.grid_forget()
            self.api_endpoint_entry.grid_forget()

        self.camera_label.grid(row=3, column=0)
        self.cameraselect.grid(row=3, column=1)

    def select_path(self):
        path_ = filedialog.askopenfilename(initialdir=DEFAULT_MODEL_PATH)
        if path_ == "":
            # 当打开文件路径选择框后点击"取消" 输入框会清空路径，所以使用get()方法再获取一次路径
            path_ = self.path.get()
        elif os.name == "nt":  # 如果是 Windows 系统
            path_ = path_.replace("/", "\\")  # 实际在代码中执行的路径为“\“ 所以替换一下
        self.path.set(path_)
        LOG.info(f"选择的模型路径为：{path_}")

    def set_chess_think_depth(self, value):
        self.chess_think_depth_label.config(
            text="象棋AI思考步数，步数越多，难度越高，超过5步会等较长时间。当前思考步数："
            + value
        )
        self.chess_think_depth = int(value)

    def detect(self):
        pre_img = self.get_video_frame()  # 获取初始视频帧
        last_class_list = []
        focus_finder = FocusFinder()
        # 启动安全检测线程
        safe_thread = threading.Thread(target=self.safe_detect)
        safe_thread.setDaemon(True)
        safe_thread.start()

        # 采集图像
        # todo：是否会太频繁
        while self.detect_flag:
            # start_time = time.time()
            cur_img = self.get_video_frame()
            if cur_img is None:
                continue
            tk_show_img_opencv_only(self.panel, cur_img, suffix="[origin image]")

            diff = cv2.absdiff(cur_img, pre_img)
            max_diff = np.max(diff)
            pre_img = cur_img
            # 稳定性前置过滤，确保抓取到稳定图像
            if max_diff > 180:
                # LOG.debug(f"相邻两帧像素差异过大，画面不稳定:{max_diff}")
                continue

            if self.model_type.get() == "llm":
                # 这里添加LLM的图像分析逻辑，调用LLM模型进行分析，并发布结果
                result = self.llm_recognizer.detect_chess_pieces(
                    cur_img, model="doubao-seed-2-0-lite-260215"
                )
                if result:
                    self.publish("llm_res", result["pieces"])
                pass
            else:
                # 取最大轮廓，即棋盘
                focus_image, has_res = focus_finder.find_chessboard(cur_img)
                if has_res:
                    # tk_show_img_opencv_only(self.panel, focus_image, suffix="[focus image]")
                    # res_img: 经过检测标注的图像（可能在目标周围绘制了边界框）
                    # yolo_list: 检测到的目标列表，包含了检测到的对象类别、位置等信息
                    res_img, yolo_list = self.self_yolo.detect(focus_image)
                    tk_show_img_opencv_only(self.panel, res_img, suffix="[yolo detect]")
                    # 将YOLO输出的归一化坐标转换为像素坐标
                    pixel_list = yolo_to_pixel(
                        yolo_list, res_img.shape[0], res_img.shape[1]
                    )

                    # 按类型排序，如果相邻两帧的检测结果相同，则认为是可信的
                    pixel_list.sort(key=lambda x: x[2], reverse=False)
                    new_class_list = [i[2] for i in pixel_list]
                    if len(new_class_list) == 0:
                        # LOG.debug(f"目标检测结果为空:{new_class_list}")
                        continue
                    is_stable = False
                    if new_class_list == last_class_list:
                        # LOG.debug(f"可信的目标检测结果:{new_class_list}")
                        # LOG.debug(f"可信的目标检测结果:{pixel_list}")
                        is_stable = True
                    last_class_list = new_class_list

                    if self.connect_flag and is_stable:
                        # 发布检测结果，供机器人控制模块使用
                        self.publish("yolo_res", pixel_list, res_img.shape)
                        LOG.debug(f"publish yolo_res to robot")

            # LOG.debug(f"本帧运行时间:{time.time() - start_time}")
            time.sleep(0.2)
        if self.detect_flag:
            self.close()

    def safe_detect(self):
        file_path = self.data_path
        data = YamlHandler(file_path).read_yaml()
        name = data["names"]
        if "hand" not in name:
            return
        hand_class = data["names"].index("hand")
        last_stop_time = 0
        count = 0
        pause_flag = False
        while self.detect_flag:
            hand_flag = False
            if self.robot_master is not None and self.robot_master.is_working():
                cur_img = self.get_video_frame()
                res_img, yolo_list = self.self_yolo.detect(cur_img)
                # tk_show_img(self.panel, res_img)
                # LOG.debug(f"人手检测结果:{yolo_list}")
                for _, _, _, _, c in yolo_list:
                    if c == hand_class:
                        count += 1
                        if count >= 2:
                            count = 0
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


if __name__ == "__main__":
    root = tkinter.Tk()
    DetecterWindow(root)
    root.mainloop()
