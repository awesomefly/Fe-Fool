import time

import numpy as np
from windows.window_robot import NINE_POINT, ENGINE_NUM
from robot.chess import Chess
from robot.robot_master import ChessRobotMaster
from robot import SERVER_ADDR, ROOT, LOG
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    # ai_move = 'f7f5'

    # chess = Chess()
    # ai_pick_pos, ai_down_pos = chess.str_to_pos(ai_move)

    # robot_master = ChessRobotMaster(think_depth=3)  # 象棋

    # ai_pick_pos_x, ai_pick_pos_y = ai_pick_pos
    # ai_pick_coordinate_x, ai_pick_coordinate_y = robot_master.pos_to_coordinate(
    #     ai_pick_pos_x, ai_pick_pos_y
    # )
    # # move,26.485471725463867,20.37872076034546,
    # print(
    #     f"ai_pick_coordinate_x: {ai_pick_coordinate_x}, ai_pick_coordinate_y: {ai_pick_coordinate_y}"
    # )

    # ai_down_pos_x, ai_down_pos_y = ai_down_pos
    # ai_down_coordinate_x, ai_down_coordinate_y = robot_master.pos_to_coordinate(
    #     ai_down_pos_x, ai_down_pos_y
    # )
    # print(
    #     f"ai_down_coordinate_x: {ai_down_coordinate_x}, ai_down_coordinate_y: {ai_down_coordinate_y}"
    # )

    pixel_list1 = [
        [648.3471871614456, 819.2659288644791, 2],
        [546.4571474790573, 1046.0595573186874, 6],
        [531.0775458216667, 636.1664832830429, 8],
        [422.9395697712898, 276.7298303693533, 9],
        [655.0757291316986, 68.66228796169162, 12],
        [542.1316335201263, 70.2227927222848, 13],
    ]
    img_shape1 = (1066, 1109, 3)

    pixel_list2 = [
        [642.220475256443, 812.2619407176971, 2],
        [537.8295716643333, 1038.3233622908592, 6],
        [524.8408566713333, 627.255399286747, 8],
        [544.5644891858101, 258.8013049811125, 9],
        [651.8417314887047, 67.55857383832335, 12],
        [539.2727410197258, 69.63729823380709, 13],
    ]
    img_shape2 = (1067, 1109, 3)

    robot_master = ChessRobotMaster(think_depth=3)  # 象棋
    robot_master.work(pixel_list1, img_shape1)

    robot_master.work(pixel_list2, img_shape2)
