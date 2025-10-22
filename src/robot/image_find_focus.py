# -*- coding: utf-8 -*-
import cv2
import numpy as np
from robot import LOG


class FocusFinder(object):
    def __init__(self, scale=1, allowed_moving_girth=300):
        self.scale = scale
        self.allowed_moving_girth = allowed_moving_girth
        self.allowed_moving_length = 80
        self.pre_corner_point = [(0, 0), (0, 0), (0, 0), (0, 0)]
        self.pre_max_length = 0
        self.is_first = True

    def find_focus(self, img, min_threshold=30, max_threshold=250):
        # 保存原始图像副本，用于后续的透视变换
        source_img = img.copy()
        # 对图像进行高斯模糊处理，减少噪声影响
        img = cv2.GaussianBlur(img, (3, 3), 0, 0)

        # 使用Canny算法进行边缘检测
        canny = cv2.Canny(img, min_threshold, max_threshold)
        # cv2.imshow("canny1", canny)
        # 创建3x3的全1矩阵作为形态学操作的核
        # 结构元素过大可能会造成过度
        k = np.ones((1, 1), np.uint8)
        # 对边缘图像进行闭运算，连接断开的边缘
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, k)
        # cv2.imshow("canny2", canny)

        # 查找边缘图像的外部轮廓
        contours, hierarchy = cv2.findContours(
            canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # 按轮廓面积从大到小排序，只保留面积最大的一个轮廓
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        # 如果没有找到轮廓，返回失败
        if len(contours) == 0:
            LOG.debug("未找到轮廓")
            return 0, False
        # 计算最大轮廓的周长
        max_length = abs(cv2.arcLength(contours[0], True))

        # 如果轮廓周长小于阈值600，认为不是有效的聚焦区域，返回失败
        if max_length < 200:
            LOG.debug(f"轮廓周长为{max_length},小于100")
            return 0, False
        # 创建与边缘图像相同尺寸的白色画布
        temp_caver = np.ones(canny.shape, np.uint8) * 255
        # 轮廓逼近函数，用于将复杂的轮廓近似为具有较少顶点的简单多边形。
        # 这在图像处理中常用于简化轮廓形状，去除不必要的细节，使轮廓更加规整。
        contours1 = cv2.approxPolyDP(contours[0], 2, True)
        # 在画布上绘制逼近后的轮廓
        cv2.drawContours(temp_caver, contours1, -1, (0, 255, 0), 1)

        debug_caver = np.ones((canny.shape[0], canny.shape[1], 3), np.uint8) * 255
        cv2.drawContours(debug_caver, contours[0], -1, (0, 0, 0), 1)  # black
        cv2.drawContours(debug_caver, contours1, -1, (0, 255, 0), 2)  # green

        # 使用Shi-Tomasi算法检测角点作为备选角点
        corners = cv2.goodFeaturesToTrack(temp_caver, 25, 0.5, 10)
        # 如果没有检测到角点，返回失败
        if corners is None:
            LOG.debug("未检测到角点")
            return 0, False
        else:
            for corner in corners:
                x, y = corner.ravel()  # 获取角点坐标
                # 在角点位置绘制红色圆圈，半径为3，线宽为-1（填充）
                cv2.circle(debug_caver, (int(x), int(y)), 3, (0, 0, 255), 1)  # red
        # 设置角点精化算法的终止条件
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # 对检测到的角点进行亚像素级精化
        cv2.cornerSubPix(temp_caver, corners, (11, 11), (-1, -1), criteria)
        # 将角点坐标转换为整数类型
        corners = np.int0(corners)
        if len(corners) < 4:
            LOG.debug(f"角点数量:{len(corners)}小于 4 个")
            return 0, False

        # # 提取角点坐标到列表中
        # point_list = []
        # for i in corners:
        #     x, y = i.ravel()
        #     point_list.append((x, y))
        # # 调用find_corner方法找到4个关键顶点
        # corner_point = self.find_corner(point_list)  # 找到4个顶点

        corner_point = self.find_corner3(corners)  # 找到4个顶点
        LOG.debug(f"角点位置:{corner_point}")
        # 调用sort_corner方法对4个顶点进行排序
        sort_corner_list = self.sort_corner(corner_point)
        for x, y in sort_corner_list:
            cv2.circle(debug_caver, (int(x), int(y)), 5, (255, 0, 0), 1)  # blue
        # cv2.imshow("debug Caver", debug_caver)

        # 如果是第一帧，初始化参考角点和参考周长
        if self.is_first:
            self.pre_corner_point = sort_corner_list
            self.pre_max_length = max_length
            self.is_first = False

        # 检测物体是否移动过大（通过周长变化判断）
        if abs(self.pre_max_length - max_length) > self.allowed_moving_girth:
            LOG.debug("物体位置移动过大")
            self.pre_max_length = max_length
            return 0, False

        # 检测角点是否变化过大（通过角点位置变化判断）
        if (
            np.max(abs(np.array(sort_corner_list) - np.array(self.pre_corner_point)))
            > self.allowed_moving_length
        ):
            LOG.debug(f"角点位置变化过大:{sort_corner_list}, {self.pre_corner_point}")
            self.pre_corner_point = sort_corner_list
            return 0, False
        # 更新参考角点和参考周长
        self.pre_corner_point = sort_corner_list
        self.pre_max_length = max_length

        # 计算目标图像的尺寸
        hight, width = self.calSize(sort_corner_list, self.scale)
        # 定义目标图像的四个角点坐标
        aim_size = np.float32([[0, 0], [width, 0], [width, hight], [0, hight]])
        # 提取原始图像中检测到的四个角点坐标
        raw_size = []
        for x, y in sort_corner_list:
            raw_size.append([x, y])
        # 将坐标转换为浮点数类型
        raw_size = np.float32(raw_size)
        # 计算透视变换矩阵
        translate_map = cv2.getPerspectiveTransform(raw_size, aim_size)
        # 应用透视变换，将图像中任意四边形区域矫正为规则矩形
        translate_img = cv2.warpPerspective(
            source_img, translate_map, (int(width), int(hight))
        )
        # 对图像进行水平翻转（镜像处理）
        translate_img = cv2.flip(translate_img, 1)  # 对角镜像
        # 返回矫正后的图像和成功标志
        return translate_img, True

    def calSize(self, sort_corner_list, scale):
        h1 = sort_corner_list[2][1] - sort_corner_list[1][1]
        h2 = sort_corner_list[3][1] - sort_corner_list[0][1]
        hight = max(h1, h2) * scale

        w1 = sort_corner_list[0][0] - sort_corner_list[1][0]
        w2 = sort_corner_list[3][0] - sort_corner_list[2][0]
        width = max(w1, w2) * scale

        return hight, width

    def sort_corner(self, corner_point):
        for i in range(len(corner_point)):
            for j in range(i + 1, len(corner_point)):
                if corner_point[i][1] > corner_point[j][1]:
                    tmp = corner_point[j]
                    corner_point[j] = corner_point[i]
                    corner_point[i] = tmp
        top = corner_point[:2]
        bot = corner_point[2:]

        if top[0][0] > top[1][0]:
            tmp = top[1]
            top[1] = top[0]
            top[0] = tmp

        if bot[0][0] > bot[1][0]:
            tmp = bot[1]
            bot[1] = bot[0]
            bot[0] = tmp

        tl = top[1]
        tr = top[0]
        bl = bot[0]
        br = bot[1]
        corners = [tl, tr, bl, br]
        return corners

    def area(self, a, b, c):
        return (a[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (a[1] - c[1])

    # 使用了一种基于直线分割的方法来寻找四个顶点，试图最大化两侧点到直线的距离之和
    def find_corner(self, point_list):
        corner_num = len(point_list)
        ans = 0.0
        ans_point_index_list = [0, 0, 0, 0]
        m1_point = 0
        m2_point = 0
        for i in range(corner_num):
            for j in range(corner_num):
                if i == j:
                    continue
                m1 = 0.0
                m2 = 0.0

                for k in range(corner_num):
                    if k == i or k == j:
                        continue
                    a = point_list[i][1] - point_list[j][1]
                    b = point_list[j][0] - point_list[i][0]
                    c = (
                        point_list[i][0] * point_list[j][1]
                        - point_list[j][0] * point_list[i][1]
                    )
                    temp = a * point_list[k][0] + b * point_list[k][1] + c

                    if temp > 0:
                        tmp_area = abs(
                            self.area(point_list[i], point_list[j], point_list[k]) / 2
                        )
                        if tmp_area > m1:
                            m1 = tmp_area
                            m1_point = k

                    elif temp < 0:
                        tmp_area = abs(
                            self.area(point_list[i], point_list[j], point_list[k]) / 2
                        )
                        if tmp_area > m2:
                            m2 = tmp_area
                            m2_point = k

                if m1 == 0.0 or m2 == 0.0:
                    continue
                if m1 + m2 > ans:
                    ans_point_index_list[0] = i
                    ans_point_index_list[1] = j
                    ans_point_index_list[2] = m1_point
                    ans_point_index_list[3] = m2_point
                    ans = m1 + m2
        ans_point_list = []
        for i in ans_point_index_list:
            ans_point_list.append(point_list[i])
        return ans_point_list

    def find_corner2(self, point_list):
        """
        从圆形坐标数组中找到最边缘的四个顶点
        简化版本：直接使用极值点
        """
        if len(point_list) < 4:
            # 如果点数少于4个，复制现有点以确保有4个点
            if len(point_list) == 0:
                # 如果没有点，返回默认值
                return [(0, 0), (10, 0), (10, 10), (0, 10)]
            elif len(point_list) == 1:
                # 如果只有1个点，复制4次
                return [point_list[0], point_list[0], point_list[0], point_list[0]]
            elif len(point_list) == 2:
                # 如果只有2个点，复制2次
                return [point_list[0], point_list[1], point_list[0], point_list[1]]
            elif len(point_list) == 3:
                # 如果只有3个点，复制第1个点
                return [point_list[0], point_list[1], point_list[2], point_list[0]]

        # 转换为numpy数组便于计算
        points = np.array(point_list)

        # 找到极值点索引
        leftmost_idx = np.argmin(points[:, 0])
        rightmost_idx = np.argmax(points[:, 0])
        topmost_idx = np.argmin(points[:, 1])
        bottommost_idx = np.argmax(points[:, 1])

        # 返回四个极值点
        return [
            point_list[leftmost_idx],
            point_list[topmost_idx],
            point_list[rightmost_idx],
            point_list[bottommost_idx],
        ]

    def find_corner3(self, point_list):
        # 最小外接矩形
        rect = cv2.minAreaRect(
            point_list
        )  # returns (center (x,y), (width, height), angle)
        box = cv2.boxPoints(rect)  # 4个点的坐标
        box = np.int32(box)

        point_list = []
        for i in box:
            x, y = i.ravel()
            point_list.append((x, y))
        return point_list
