# -*- coding: utf-8 -*-
import random
import socket
from tkinter import messagebox
import time
from abc import abstractmethod
from math import sqrt

from robot.tools import coordinate_mapping, play_sound, play_sound_thread, get_name_by_class, \
    plane_coordinate_transform, YamlHandler, Observer, GlobalVar
from robot import LOG, PARAMS_YAML, SERVER_ADDR, chess, gobang
from robot.robot_ik import inverse_kinematics

PARAMS = YamlHandler(PARAMS_YAML).read_yaml()  # 读取params.yaml文件中配置的参数

"""
（这里用中文会错位，见谅）
            ^  y
    Camera  |
            |
    Board   |  Robot 
   <--------|
   x
（注意，这里只是标注方向，坐标原点为机械臂底座中点）

宽:x方向
长:y方向
高:z方向，桌面的法向量
"""

# 普通抓取模式工作台参数,读取用户自定义的参数
WIDTH_GRAB = PARAMS['width_grab']  # 工作台宽，x方向
LENGTH_GRAB = PARAMS['length_grab']  # 工作台长，y方向
HIGH_GRAB = PARAMS['high_grab']  # 工作台高，z方向
TRANSFORM_X_GRAB = PARAMS['transform_x_grab']  # 棋盘距离机械臂原点X轴的平移
MAP_CLASS_2_POS = PARAMS['map_class_2_pos']  # key:目标种类  value:抓取目标点

# 五子棋物理参数
WIDTH_GOBANG = 234  # 五子棋盘总宽度
LENGTH_GOBANG = 250  # 五子棋盘总长度
HIGH_GOBANG = 2  # 五子棋棋盘高度
WIDTH_ERR_GOBANG = 23  # 五子棋盘内外边框间距(宽度方向)
LENGTH_ERR_GOBANG = 25  # 五子棋盘内外边框间距(长度方向)
ROW_GOBANG = 9  # 五子棋盘行数(宽度方向)
COLUMN_GOBANG = 9  # 五子棋盘列数(长度方向)

TEMP = PARAMS['pick_point']
PICK_POINT_GOBANG = [TEMP[0], TEMP[1], 5]  # 取五子棋的固定点
MID_POINT_GOBANG = [TEMP[0], 50, 50]  # 取五子棋的后走的中间点
START_POINT_GOBANG = [TEMP[0], TEMP[1], 40]  # 五子棋模式固定起始点

# 象棋物理参数
WIDTH_CHESS = 200  # 象棋盘总宽度
LENGTH_CHESS = 200  # 象棋盘总长度
HIGH_CHESS = 20  # 象棋棋盘高度
WIDTH_ERR_CHESS = 20  # 象棋盘内外边框间距(宽度方向)
LENGTH_ERR_CHESS = 18  # 象棋盘内外边框间距(长度方向)

CHESS_DUMP_COORDINATE = [120, -190, 40]  # 象棋吃子后放置的固定点
START_POINT_CHESS = [50, 100, 30]  # 象棋模式固定起始点

# 其他公共参数
TRANSFORM_X = 120  # 棋盘/工作台距离机械臂原点X轴的平移
ERROR_NUM = 9999  # 一个较大的值来做为错误值


class RobotMaster(Observer):
    def __init__(self, width, length):
        self.width = width
        self.length = length
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.start_flag = False
        self.pause_flag = False
        self.start_point = []

    # window_detection发布过来的消息在这里处理
    def receive_message(self, topic, message):
        LOG.debug(f"received message:{topic}, {message}")
        if topic == "safety" and self.is_start():
            if message[0] == "pause":
                self.pause()
            elif message[0] == "resume":
                self.resume()
        elif topic == "yolo_res":
            if len(message) == 2:
                self.work(message[0], message[1])

    @abstractmethod
    def work(self, pixel_list, img_shape):
        pass

    def pause(self):
        if not self.pause_flag:
            self.pause_flag = True
            self.client.send('pause\n'.encode())
            play_sound("hand")

    def resume(self):
        if self.pause_flag:
            self.pause_flag = False
            self.client.send('resume\n'.encode())

    def connect_robot(self):
        res = self.client.connect_ex(SERVER_ADDR)  # 连接机械臂模块
        if res == 0:
            self.robot_back()
        return res

    def close(self):
        self.client.close()

    def is_start(self):
        return self.start_flag

    # mod: 1 去某点取   2  放到某点
    def send_command(self, str_command):
        while True:
            self.client.send(str_command.encode())  # 发送数据
            buffer = self.client.recv(1024).decode()  # 接收数据
            LOG.debug(f"返回数据:{buffer}")
            if buffer == "done":
                break

    def command_to_str(self, mod, *args):
        res_str = mod + ','
        for n in args:
            res_str += str(n) + ","
        return res_str

    def robot_move_to(self, start_coordinate, target_coordinate, mid_coordinate=[]):
        # start_coordinate取子，从目标位置高8mm的位置开始下降，这样的控制会更加稳定
        self.send_command(self.command_to_str("move", start_coordinate[0], start_coordinate[1],
                                              start_coordinate[2] + 8))
        time.sleep(1.5)
        self.send_command(self.command_to_str("move", start_coordinate[0], start_coordinate[1], start_coordinate[2]))
        self.send_command(self.command_to_str("pick"))

        # 没设置中间点抬起30mm
        if len(mid_coordinate) != 3:
            self.send_command(
                self.command_to_str("move", start_coordinate[0], start_coordinate[1], start_coordinate[2] + 20))
        # 设置中间点往中间点走
        else:
            self.send_command(self.command_to_str("move", mid_coordinate[0], mid_coordinate[1], mid_coordinate[2]))

        # 目标点放
        self.send_command(
            self.command_to_str("move", target_coordinate[0], target_coordinate[1], target_coordinate[2] + 8))
        time.sleep(1.5)
        self.send_command(self.command_to_str("move", target_coordinate[0], target_coordinate[1], target_coordinate[2]))
        self.send_command(self.command_to_str("down"))
        # 抬起20mm
        self.send_command(
            self.command_to_str("move", target_coordinate[0], target_coordinate[1], target_coordinate[2] + 20))

    def robot_back(self):
        # 回到起点
        self.send_command(self.command_to_str("move", self.start_point[0], self.start_point[1], self.start_point[2]))

    def is_robot_has_ik(self, coordinate):
        coordinate_x, coordinate_y = plane_coordinate_transform(coordinate_x=coordinate[0],
                                                                coordinate_y=coordinate[1],
                                                                transform_x=TRANSFORM_X,
                                                                transform_y=-self.length / 2,
                                                                transform_angle=0)
        has_ik1, _, _, _ = inverse_kinematics(coordinate_x, coordinate_y, coordinate[2])
        has_ik2, _, _, _ = inverse_kinematics(coordinate_x, coordinate_y, coordinate[2] + 20)
        return has_ik1 & has_ik2


class BoardGamesRobotMaster(RobotMaster):
    def __init__(self, width, length, width_err, length_err):
        super().__init__(width, length)
        self.length_err = length_err
        self.width_err = width_err
        self.history_set = set()
        self.count = 0
        self.last_down_time = 0
        self.last_wring_time = 0
        self.overtime = 20  # 超时提醒
        self.our_class_list = []  # 玩家使用的棋子类型

    @abstractmethod
    def check_start(self, coordinate_list):
        pass

    @abstractmethod
    def coordinate_to_pos(self, coordinate_list):
        pass

    @abstractmethod
    def find_last_down_pos(self, pos_set):
        pass

    @abstractmethod
    def pos_to_coordinate(self, x, y):
        pass

    @abstractmethod
    def restart(self):
        pass

    @abstractmethod
    # 获取我方棋子种类
    def get_our_class(self):
        pass


class GobangRobotMaster(BoardGamesRobotMaster):
    def __init__(self, width=WIDTH_GOBANG, length=LENGTH_GOBANG, width_err=WIDTH_ERR_GOBANG,
                 length_err=LENGTH_ERR_GOBANG,
                 row=ROW_GOBANG, column=COLUMN_GOBANG):
        super().__init__(width, length, width_err, length_err)
        self.row = row
        self.column = column
        self.start_point = START_POINT_GOBANG
        self.gobang_ai = gobang.Gobang(row=self.row, column=self.column)
        self.get_our_class()

    # 玩家使用的五子棋类别，黑子(先手)
    def get_our_class(self):
        file_path = GlobalVar.get_value('DATA_YAML_PATH')
        data = YamlHandler(file_path).read_yaml()
        name = data['names']
        if 'heizi' in name:
            self.our_class_list.append(data['names'].index('heizi'))
        else:
            messagebox.showerror('错误', '该模型未包含五子棋或种类出错，不可使用五子棋模式')

    def restart(self):
        answer = messagebox.askokcancel('憨憨', '再来一局？请在确认前收好棋子')
        if answer:
            self.gobang_ai = gobang.Gobang(row=self.row, column=self.column)
            self.history_set = set()
            self.count = 0
            self.last_down_time = 0
            self.start_flag = False
        else:
            self.close()

    def check_start(self, coordinate_list):
        if not self.start_flag:
            if len(coordinate_list) == 0:
                play_sound_thread("start")
                self.start_flag = True
                self.last_down_time = time.time()
                return True
            else:
                # 防止机器人一直叫唤
                if time.time() - self.last_wring_time > 8:
                    play_sound("wrong_start")
                    self.last_wring_time = time.time()
                return False

        else:
            return True

    # pixel:像素坐标  coordinate:物理坐标  pos:棋盘行列位置
    def work(self, pixel_list, img_shape):
        coordinate_list = coordinate_mapping(pixel_list, self.width, self.length, img_shape[0], img_shape[1])
        if not self.check_start(coordinate_list):
            return
        pos_set = self.coordinate_to_pos(coordinate_list)
        our_down_pos = self.find_last_down_pos(pos_set)

        if time.time() - self.last_down_time > self.overtime:
            self.last_down_time = time.time()
            play_sound_thread("overtime")
        if our_down_pos:
            self.last_down_time = time.time()
            our_down_pos_x, our_down_pos_y = our_down_pos
            ai_down_pos_x, ai_down_pos_y, res = self.gobang_ai.down(our_down_pos_x, our_down_pos_y)
            if res == -1:
                return
            else:
                ai_down_coordinate_x, ai_down_coordinate_y = self.pos_to_coordinate(ai_down_pos_x, ai_down_pos_y)
                # 先判断机械臂有没有解
                if not self.is_robot_has_ik((ai_down_coordinate_x, ai_down_coordinate_y, HIGH_GOBANG)):
                    play_sound_thread("res")
                    return

                self.history_set.add(our_down_pos)
                self.robot_do_gobang(ai_down_coordinate_x, ai_down_coordinate_y)

                if res == 1:  # 胜负已分
                    play_sound_thread('win')
                    self.restart()
                if res == 2:
                    play_sound_thread('lost')
                    self.restart()
                else:
                    play_sound_thread("your")
                    self.last_down_time = time.time()

    def robot_do_gobang(self, ai_down_coordinate_x, ai_down_coordinate_y):
        # 棋盘坐标转机械臂坐标
        ai_down_coordinate_x, ai_down_coordinate_y = plane_coordinate_transform(coordinate_x=ai_down_coordinate_x,
                                                                                coordinate_y=ai_down_coordinate_y,
                                                                                transform_x=TRANSFORM_X,
                                                                                transform_y=-self.length / 2,
                                                                                transform_angle=0)

        self.robot_move_to(PICK_POINT_GOBANG, (ai_down_coordinate_x, ai_down_coordinate_y, HIGH_GOBANG),
                           MID_POINT_GOBANG)
        self.robot_back()

    def coordinate_to_pos(self, coordinate_list):
        pos_set = set()
        for coordinate_x, coordinate_y, c in coordinate_list:
            if c in self.our_class_list:  # 只计算玩家的棋子
                pos_x = round(
                    abs(coordinate_x - self.width_err) / (self.width - 2 * self.width_err) * (self.column - 1))
                pos_y = round(
                    abs(coordinate_y - self.length_err) / (self.length - 2 * self.length_err) * (self.row - 1))
                pos_set.add((pos_x, pos_y))
        return pos_set

    def find_last_down_pos(self, now_pos_set):
        our_down_pos = now_pos_set - self.history_set
        LOG.info(f"玩家最后一次落子落子位置:{our_down_pos}")
        if len(our_down_pos) == 1:
            self.count += 1
            if self.count == 3:
                self.count = 0
                our_down_pos_x, our_down_pos_y = list(our_down_pos)[0]
                if our_down_pos_x >= self.row or our_down_pos_y >= self.column:
                    return None
                return our_down_pos_x, our_down_pos_y
        return None

    def pos_to_coordinate(self, x, y):
        x = self.width_err + x * (self.width - 2 * self.width_err) / (self.column - 1)
        y = self.length_err + y * (self.length - 2 * self.length_err) / (self.row - 1)
        return x, y


class ChessRobotMaster(BoardGamesRobotMaster):
    def __init__(self, width=WIDTH_CHESS, length=LENGTH_CHESS, width_err=WIDTH_ERR_CHESS, length_err=LENGTH_ERR_CHESS):
        super().__init__(width, length, width_err, length_err)
        self.chess_ai = chess.Chess()
        self.all_chess_list = []
        self.start_point = START_POINT_CHESS
        self.get_our_class()

    # 玩家的象棋类别，红子(先手)
    def get_our_class(self):
        file_path = GlobalVar.get_value('DATA_YAML_PATH')
        data = YamlHandler(file_path).read_yaml()
        index = 0
        for name in data['names']:
            if name.startswith('hong'):
                self.our_class_list.append(index)
            index = index + 1

        if len(self.our_class_list) != 7:
            messagebox.showerror('错误', '该模型未包含象棋或种类出错，不可使用象棋模式')

    def restart(self):
        answer = messagebox.askokcancel('憨憨', '再来一局？')
        if answer:
            self.chess_ai = chess.Chess()
            self.history_set.clear()
            self.count = 0
            self.last_down_time = 0
            self.start_flag = False
        else:
            self.close()

    # 这里为了不与棋子种类耦合，这里就这么麻烦且不准确的判断了
    def check_start(self, coordinate_list):
        if not self.start_flag:
            if len(coordinate_list) == 32:
                map_temp = self.get_map_pos_2_class(coordinate_list)
                if map_temp[0][0] == map_temp[8][0] and map_temp[0][9] == map_temp[8][9] and map_temp[1][0] == \
                        map_temp[7][0] and map_temp[1][9] == map_temp[7][9] and map_temp[2][0] == map_temp[6][0] and \
                        map_temp[2][9] == map_temp[6][9] and map_temp[3][0] == map_temp[5][0] and map_temp[3][9] == \
                        map_temp[5][9] and map_temp[1][2] == map_temp[7][2] and map_temp[1][7] == map_temp[7][7] and \
                        map_temp[0][3] == map_temp[2][3] == map_temp[4][3] == map_temp[6][3] == map_temp[8][3] and \
                        map_temp[0][6] == map_temp[2][6] == map_temp[4][6] == map_temp[6][6] == map_temp[8][6]:
                    play_sound_thread("start")
                    self.last_down_time = time.time()
                    self.start_flag = True
                    return True
            # 防止机器人一直叫唤
            if time.time() - self.last_wring_time > 8:
                play_sound("wrong_start")
                LOG.error(f"{time.time() - self.last_wring_time}")
                self.last_wring_time = time.time()
            return False
        else:
            return True

    def work(self, pixel_list, img_shape):
        coordinate_list = coordinate_mapping(pixel_list, self.width, self.length, img_shape[0], img_shape[1])
        if not self.check_start(coordinate_list):
            return
        pos_set = self.coordinate_to_pos(coordinate_list)
        pos = self.find_last_down_pos(pos_set)

        if time.time() - self.last_down_time > self.overtime:
            self.last_down_time = time.time()
            play_sound_thread("overtime")
        self.all_chess_list = coordinate_list
        if pos:
            self.last_down_time = time.time()

            our_pick_pos, our_down_pos = pos
            ai_pick_pos, ai_down_pos, res = self.chess_ai.player_down(our_pick_pos[0:-1], our_down_pos[0:-1])
            if res == -1:
                play_sound_thread('wrong_down')
                return
            else:
                ai_pick_pos_x, ai_pick_pos_y = ai_pick_pos
                ai_pick_coordinate_x, ai_pick_coordinate_y = self.pos_to_coordinate(ai_pick_pos_x, ai_pick_pos_y)
                # 先判断机械臂有没有解
                if not self.is_robot_has_ik((ai_pick_coordinate_x, ai_pick_coordinate_y, HIGH_CHESS)):
                    play_sound_thread("res")
                    return

                ai_down_pos_x, ai_down_pos_y = ai_down_pos
                ai_down_coordinate_x, ai_down_coordinate_y = self.pos_to_coordinate(ai_down_pos_x, ai_down_pos_y)
                # 先判断机械臂有没有解
                if not self.is_robot_has_ik((ai_down_coordinate_x, ai_down_coordinate_y, HIGH_CHESS)):
                    play_sound_thread("res")
                    return

                self.history_set.add(our_down_pos)
                self.history_set.remove(our_pick_pos)

                is_eat = False
                for pos in self.history_set:
                    if ai_down_pos == pos[0:-1]:
                        is_eat = True
                        self.history_set.remove(pos)
                        break

                self.robot_do_chess(ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x,
                                    ai_down_coordinate_y, is_eat)

                if res == 1:  # 胜负已分
                    play_sound_thread('win')
                    self.restart()
                elif res == 2:
                    play_sound_thread('lost')
                    self.restart()
                elif res == 3:
                    play_sound_thread('checkmate')
                else:
                    play_sound_thread("your")
                    self.last_down_time = time.time()

    def robot_do_chess(self, ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x, ai_down_coordinate_y,
                       is_eat):
        # 这里算出来的是棋盘中交点的坐标，应该去取棋子的实际坐标
        ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x, ai_down_coordinate_y = self.find_real_coordinate(
            ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x, ai_down_coordinate_y,
            is_eat)
        if ai_pick_coordinate_x == 0:
            play_sound_thread("chess_wrong")
            return
        # 棋盘坐标转机械臂坐标
        ai_pick_coordinate_x, ai_pick_coordinate_y = plane_coordinate_transform(coordinate_x=ai_pick_coordinate_x,
                                                                                coordinate_y=ai_pick_coordinate_y,
                                                                                transform_x=TRANSFORM_X,
                                                                                transform_y=-self.length / 2,
                                                                                transform_angle=0)

        ai_down_coordinate_x, ai_down_coordinate_y = plane_coordinate_transform(coordinate_x=ai_down_coordinate_x,
                                                                                coordinate_y=ai_down_coordinate_y,
                                                                                transform_x=TRANSFORM_X,
                                                                                transform_y=-self.length / 2,
                                                                                transform_angle=0)
        if is_eat:  # 需要吃子
            # 随机范围放置，防止堆太高
            random_dump_coordinate = [CHESS_DUMP_COORDINATE[0] + random.randint(-25, 25),
                                      CHESS_DUMP_COORDINATE[1] + random.randint(-25, 25), CHESS_DUMP_COORDINATE[2]]
            self.robot_move_to((ai_down_coordinate_x, ai_down_coordinate_y, HIGH_CHESS), random_dump_coordinate)
        self.robot_move_to((ai_pick_coordinate_x, ai_pick_coordinate_y, HIGH_CHESS),
                           (ai_down_coordinate_x, ai_down_coordinate_y, HIGH_CHESS))
        self.robot_back()

    def find_real_coordinate(self, ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x,
                             ai_down_coordinate_y,
                             is_eat):
        if len(self.all_chess_list) == 0:
            return 0, 0, 0, 0
        if is_eat:
            index = self.find_min_dis_index(self.all_chess_list, ai_down_coordinate_x, ai_down_coordinate_y)
            if index == ERROR_NUM:
                return 0, 0, 0, 0
            ai_down_coordinate_x, ai_down_coordinate_y, chess_class = self.all_chess_list[index]
            play_sound_thread('eat', get_name_by_class(chess_class))

        index = self.find_min_dis_index(self.all_chess_list, ai_pick_coordinate_x, ai_pick_coordinate_y)
        if index == ERROR_NUM:
            return 0, 0, 0, 0
        ai_pick_coordinate_x, ai_pick_coordinate_y, _ = self.all_chess_list[index]

        return ai_pick_coordinate_x, ai_pick_coordinate_y, ai_down_coordinate_x, ai_down_coordinate_y

    def find_min_dis_index(self, all_chess_list, x, y):
        min_dis = 9999
        index = ERROR_NUM
        for i in range(len(all_chess_list)):
            _x, _y, _ = all_chess_list[i]
            dis = sqrt((x - _x) ** 2 + (y - _y) ** 2)
            if dis < min_dis:
                min_dis = dis
                index = i
        if min_dis > 10:
            return ERROR_NUM
        return index

    def coordinate_to_pos(self, coordinate_list):
        pos_set = set()
        for coordinate_x, coordinate_y, c in coordinate_list:
            if c in self.our_class_list:  # 只计算玩家的棋子
                pos_x = round(abs(coordinate_x - self.width_err) / (self.width - 2 * self.width_err) * 8)
                pos_y = round(abs(coordinate_y - self.length_err) / (self.length - 2 * self.length_err) * 9)
                pos_set.add((pos_x, pos_y, c))
        return pos_set

    def get_map_pos_2_class(self, coordinate_list):
        map_pos_2_class = [[0] * 10 for i in range(10)]
        for coordinate_x, coordinate_y, c in coordinate_list:
            pos_x = round(abs(coordinate_x - self.width_err) / (self.width - 2 * self.width_err) * 8)
            pos_y = round(abs(coordinate_y - self.length_err) / (self.length - 2 * self.length_err) * 9)
            map_pos_2_class[pos_x][pos_y] = c
        return map_pos_2_class

    def pos_to_coordinate(self, x, y):
        x = self.width_err + x * (self.width - 2 * self.width_err) / 8
        y = self.length_err + y * (self.length - 2 * self.length_err) / 9
        return x, y

    def find_last_down_pos(self, now_pos_set):
        if len(self.history_set) == 0:
            self.history_set = now_pos_set
        else:
            our_pick_pos = self.history_set - now_pos_set
            our_down_pos = now_pos_set - self.history_set
            LOG.info(f"玩家最后一次落子落子位置:{our_pick_pos} 至 {our_down_pos}")
            if len(our_pick_pos) == 1 and len(our_down_pos) == 1 and list(our_pick_pos)[0][-1] == list(our_down_pos)[0][
                -1]:
                self.count += 1
                if self.count == 2:
                    self.count = 0
                    return list(our_pick_pos)[0], list(our_down_pos)[0]
        return None


class GrabRobotMaster(RobotMaster):
    def __init__(self, width=WIDTH_GRAB, length=LENGTH_GRAB):
        super().__init__(width, length)
        self.count = 0
        self.history_coordinate_list = []
        self.start_point = START_POINT_CHESS
        self.start_flag = True

    # pixel:像素坐标  coordinate:物理坐标
    def work(self, pixel_list, img_shape):
        coordinate_list = coordinate_mapping(pixel_list, self.width, self.length, img_shape[0], img_shape[1])
        if self.check_coordinate(coordinate_list):
            for coordinate_x, coordinate_y, _class in coordinate_list:
                # 只抓取用户配置了的物体
                if _class in MAP_CLASS_2_POS:
                    # 先判断机械臂有没有解
                    if not self.is_robot_has_ik((coordinate_x, coordinate_y, HIGH_GRAB)):
                        play_sound_thread("res")
                        return

                    self.robot_do_grab(coordinate_x, coordinate_y, HIGH_GRAB, _class)
                    self.robot_back()
                    break

    # 连续3帧物品位置的差异值小于工作台总宽的1/20，说明结果可信，如果识别精度不高，可以加上此步骤
    def check_coordinate(self, coordinate_list):
        return True
        # if len(self.history_coordinate_list) == 0:
        #     self.history_coordinate_list = coordinate_list
        #     return False
        # if 0 < len(self.history_coordinate_list) == len(coordinate_list):
        #     for i in range(len(self.history_coordinate_list)):
        #         temp = [self.history_coordinate_list[i][j] - coordinate_list[i][j] for j in range(2)]
        #         max_delta = map(abs, temp)
        #         if max(max_delta) < (self.width / 20):
        #             self.count += 1
        #             self.history_coordinate_list = coordinate_list
        #             if self.count == 2:
        #                 self.count = 0
        #         else:
        #             return False
        #     return True

    def robot_do_grab(self, coordinate_x, coordinate_y, coordinate_z, _class):
        # 棋盘坐标转机械臂坐标
        coordinate_x, coordinate_y = plane_coordinate_transform(coordinate_x=coordinate_x,
                                                                coordinate_y=coordinate_y,
                                                                transform_x=TRANSFORM_X_GRAB,
                                                                transform_y=-self.length / 2,
                                                                transform_angle=0)

        self.robot_move_to((coordinate_x, coordinate_y, coordinate_z), MAP_CLASS_2_POS[_class]['pos'])
