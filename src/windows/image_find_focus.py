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
        cv2.imshow("canny2", canny)

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
        contours1 = cv2.approxPolyDP(contours[0], 10, True)
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

        shape = self.detect_shape(contours[0])
        if shape == "circle" or shape == "polygon":
            corner_point = self.find_corner3(corners)  # 外接矩形法，找到4个顶点
        else:
            # 提取角点坐标到列表中
            point_list = []
            for i in corners:
                x, y = i.ravel()
                point_list.append((x, y))
            # 调用find_corner方法找到4个关键顶点
            corner_point = self.find_corner(point_list)  # 找到4个顶点
        LOG.debug(f"角点位置:{corner_point}, 形状:{shape}")

        # 调用sort_corner方法对4个顶点进行排序
        sort_corner_list = self.sort_corner(corner_point)
        for x, y in sort_corner_list:
            cv2.circle(debug_caver, (int(x), int(y)), 5, (255, 0, 0), 1)  # blue
        cv2.imshow("debug Caver", debug_caver)

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
        # for x, y in aim_size:
        #     cv2.circle(debug_caver, (int(x), int(y)), 5, (255, 0, 0), 1)  # blue
        # cv2.imshow("debug Caver", debug_caver)

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
        # translate_img = cv2.flip(translate_img, 1)  # 对角镜像
        # 返回矫正后的图像和成功标志
        return translate_img, True

    def calSize(self, sort_corner_list, scale):
        h1 = sort_corner_list[2][1] - sort_corner_list[1][1]
        h2 = sort_corner_list[3][1] - sort_corner_list[0][1]
        hight = max(h1, h2) * scale

        w1 = sort_corner_list[1][0] - sort_corner_list[0][0]
        w2 = sort_corner_list[2][0] - sort_corner_list[3][0]
        width = max(w1, w2) * scale

        return hight, width

    def sort_corner(self, corner_point):
        for i in range(len(corner_point)):
            for j in range(i + 1, len(corner_point)):
                if corner_point[i][1] > corner_point[j][1]:
                    tmp = corner_point[j]
                    corner_point[j] = corner_point[i]
                    corner_point[i] = tmp
        top = corner_point[:2]  # 前两个点（Y坐标较小，位于上方）
        bot = corner_point[2:]  # 后两个点（Y坐标较大，位于下方）

        if top[0][0] > top[1][0]:  # X小的在左，X大的在右
            tmp = top[1]
            top[1] = top[0]
            top[0] = tmp

        if bot[0][0] > bot[1][0]:
            tmp = bot[1]
            bot[1] = bot[0]
            bot[0] = tmp

        tl = top[0]
        tr = top[1]
        bl = bot[0]
        br = bot[1]

        # 扩大一些边界，避免裁剪边缘
        # delta = 15
        # tl = (tl[0] - delta, tl[1] - delta)
        # tr = (tr[0] + delta, tr[1] - delta)
        # bl = (bl[0] - delta, bl[1] + delta)
        # br = (br[0] + delta, br[1] + delta)
        corners = [tl, tr, br, bl]
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
        # 返回值： (center (x,y), (width, height), angle)
        rect = cv2.minAreaRect(point_list)
        box = cv2.boxPoints(rect)  # 4个点的坐标
        box = np.int32(box)

        point_list = []
        for i in box:
            x, y = i.ravel()
            point_list.append((x, y))
        return point_list

    def find_corner4(self, point_list):
        # 最小外接圆形
        (x, y), radius = cv2.minEnclosingCircle(point_list)
        center = (int(x), int(y))
        radius = int(radius)

        lt = (center[0] - radius, center[1] - radius)
        rt = (center[0] + radius, center[1] - radius)
        lb = (center[0] - radius, center[1] + radius)
        rb = (center[0] + radius, center[1] + radius)
        return [lt, rt, rb, lb], center, radius

    def is_all_points_inside(self, contour_points, approx_poly_points):
        """
        校验：原始轮廓的所有点是否在近似多边形内部
        """
        for point in contour_points:
            # 计算点到近似多边形的距离（>0 表示点在多边形内部）
            dist = cv2.pointPolygonTest(
                approx_poly_points,
                (float(point[0]), float(point[1])),
                measureDist=True,
            )
            if dist < 0:
                return False
        return True

    def optimal_approx_poly(self, cnt, max_epsilon_ratio=0.02):
        """
        优化：动态调整 epsilon 以找到“包容+最少顶点”的最优逼近多边形
        - cnt: 原始轮廓(N,1,2)
        - epsilon_ratio:初始 epsilon 比例（默认 0.1%,即周长的 0.1%）
        返回：最优逼近多边形(M,1,2)
        """
        # 1. 轮廓周长（用于动态调整 epsilon）
        arc_length = cv2.arcLength(cnt, closed=True)

        # 提取原始轮廓的所有离散点（格式：(N,2)）
        cnt_points = cnt.reshape(-1, 2)

        # 2. 初始化 epsilon（从最小阈值开始，逐步增大）
        epsilon_ratio = 0.001  # 初始比例（周长的 0.1%）
        best_approx = cnt  # 初始最优逼近（即原始轮廓）
        min_vertices = len(cnt)  # 初始最小顶点数（原始轮廓顶点数）

        # 3. 动态调整 epsilon，寻找“包容+最少顶点”的逼近
        while (
            epsilon_ratio <= max_epsilon_ratio
        ):  # 最大比例（避免过度失真，可按需调整）
            epsilon = epsilon_ratio * arc_length
            approx = cv2.approxPolyDP(cnt, epsilon, closed=True)  # epsilon 越大，点越少
            current_vertices = len(approx)

            # 关键校验：1. 顶点数更少；2. 所有原始点在近似多边形内部
            if current_vertices < min_vertices:
                # and self.is_all_points_inside(
                #     cnt_points, approx
                # ):
                LOG.debug(
                    f"epsilon_ratio={epsilon_ratio:.4f}, epsilon={epsilon:.2f}, "
                    f"vertices={current_vertices}"
                )
                min_vertices = current_vertices
                best_approx = approx  # 更新最优逼近

            # 增大 epsilon 比例（步长可按需调整，步长越小越精准）
            epsilon_ratio += 0.002
        return best_approx

    def detect_shape(self, contour):
        """
        根据轮廓 contour 判断形状类别。
        返回形状名称字符串，例如 'circle', 'square', 'rectangle', 'triangle' 等。
        contour 的形状应为 (N,1,2)，如 cv2.findContours 的输出之一。
        """
        approx = self.optimal_approx_poly(contour, epsilon_ratio=0.001)
        verts = len(approx)

        # 先按顶点数粗略分类
        if verts == 3:
            return "triangle"
        if verts == 4:
            return "rectangle"

        # 5或更多顶点时，进一步判断是否接近圆形
        # 使用最小外接圆的圆心和半径，以及轮廓面积与圆面积比值作为圆度判断
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        circle_area = np.pi * (radius**2)
        if circle_area > 0:
            area = cv2.contourArea(contour)
            circularity = area / circle_area  # 介于 0 到 1，接近1为圆形
        else:
            circularity = 0

        # 根据圆度阈值来判断
        if verts > 4 and circularity > 0.75:
            return "circle"

        # 其他多边形，作为兜底
        return "polygon"

    def merge_contours(self, contours, merge_distance=10):
        """
        简单的合并策略：对若干轮廓的外接矩形进行合并
        如果两个轮廓的外接矩形相距小于 merge_distance，则合并为一个轮廓
        返回合并后的轮廓列表（形态可能不是严格的轮廓，但有利于“一个物体一个轮廓”的目标）
        """
        if not contours:
            return []

        # 先将每个轮廓用其外接矩形表示
        rects = [cv2.boundingRect(cnt) for cnt in contours]

        merged = []
        used = [False] * len(contours)

        for i, cnt_i in enumerate(contours):
            if used[i]:
                continue
            x1, y1, w1, h1 = rects[i]
            # 初始合并轮廓
            merged_cnt = cnt_i
            used[i] = True
            for j in range(i + 1, len(contours)):
                if used[j]:
                    continue
                x2, y2, w2, h2 = rects[j]
                # 计算两矩形的最近距离
                dist = max(
                    0,
                    max(x2 - (x1 + w1), x1 - (x2 + w2), y2 - (y1 + h1), y1 - (y2 + h2)),
                )
                if dist <= merge_distance:
                    # 合并为一个轮廓：简单将两者拼接后再轮廓化
                    all_pts = np.vstack([merged_cnt, contours[j]])
                    # 将若干点再近似为轮廓
                    peri = cv2.arcLength(all_pts, True)
                    merged_cnt = cv2.approxPolyDP(all_pts, 0.01 * peri, True)
                    used[j] = True
                    # 更新矩形信息以便进一步合并
                    x1 = min(x1, x2)
                    y1 = min(y1, y2)
                    x2r = max(x1 + w1, x2 + w2)
                    y2r = max(y1 + h1, y2 + h2)
                    w1 = x2r - x1
                    h1 = y2r - y1
            merged.append(merged_cnt)
        return merged

    def crop_image(
        self,
        img,
        shape="circle",
        min_threshold=30,
        max_threshold=250,
        hight=None,
        width=None,
    ):
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
        LOG.debug(f"找到{len(contours)}个轮廓")

        # 轮廓合并
        contours = self.merge_contours(contours, merge_distance=1000000)

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

        # 轮廓逼近函数，用于将复杂的轮廓近似为具有较少顶点的简单多边形。
        # 这在图像处理中常用于简化轮廓形状，去除不必要的细节，使轮廓更加规整。
        # 轮廓周长/面积，用于后续阈值与近似
        # 轮廓近似为多边形，epsilon 取周长的 1-2%
        if shape == "circle":
            max_epsilon_ratio = 0.02
        else:
            max_epsilon_ratio = 0.02
        approx_poly_points = self.optimal_approx_poly(contours[0], max_epsilon_ratio)
        LOG.debug(
            f"原轮廓点数{len(contours[0])}，轮廓逼近后点数{len(approx_poly_points)}"
        )

        debug_caver = np.ones((canny.shape[0], canny.shape[1], 3), np.uint8) * 255
        cv2.drawContours(debug_caver, contours[0], -1, (0, 0, 0), 8)  # black
        cv2.drawContours(debug_caver, approx_poly_points, -1, (0, 255, 0), 8)  # green

        # # 创建与边缘图像相同尺寸的白色画布,在画布上绘制逼近后的轮廓
        # temp_caver = np.ones(canny.shape, np.uint8) * 255
        # cv2.drawContours(temp_caver, contours1, -1, (0, 255, 0), 1)
        # # 使用Shi-Tomasi算法检测角点作为备选角点
        # # 不太准！！经常把最优角点丢失
        # corners = cv2.goodFeaturesToTrack(temp_caver, 25, 0.8, 10)
        # # 如果没有检测到角点，返回失败
        # if corners is None:
        #     LOG.debug("未检测到角点")
        #     return 0, False
        # else:
        #     for corner in corners:
        #         x, y = corner.ravel()  # 获取角点坐标
        #         # 在角点位置绘制红色圆圈，半径为3，线宽为-1（填充）
        #         cv2.circle(debug_caver, (int(x), int(y)), 6, (0, 0, 255), 2)  # red
        # LOG.debug(f"Shi-Tomasi算法检测到角点数量{len(corners)}")
        # # 设置角点精化算法的终止条件
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # # 对检测到的角点进行亚像素级精化
        # cv2.cornerSubPix(temp_caver, corners, (11, 11), (-1, -1), criteria)
        # LOG.debug(f"角点精化后数量:{len(corners)}")
        # # 将角点坐标转换为整数类型
        # corners = np.int0(corners)
        # if len(corners) < 4:
        #     LOG.debug(f"角点数量:{len(corners)}小于 4 个")
        #     return 0, False

        if shape == "circle":
            corner_point, center, radius = self.find_corner4(approx_poly_points)
            source_img = self.crop_circle(source_img, center, radius - 20)
        else:
            points_list = []
            for v in approx_poly_points:
                points_list.append((v[0][0], v[0][1]))
            corner_point = self.find_corner(points_list)
            # corner_point = self.find_corner3(approx_poly_points)
        LOG.debug(f"角点位置:{corner_point}, 形状:{shape}")

        # 修正负坐标
        corner_point1 = []
        for x, y in corner_point:
            if x < 0:
                x = 0
            if y < 0:
                y = 0
            corner_point1.append((x, y))
            cv2.circle(debug_caver, (int(x), int(y)), 6, (0, 0, 255), 2)  # red

        # 调用sort_corner方法对4个顶点进行排序
        sort_corner_list = self.sort_corner(corner_point1)
        LOG.debug(f"排序前：{corner_point1},排序后角点位置:{sort_corner_list}")
        for x, y in sort_corner_list:
            cv2.circle(debug_caver, (int(x), int(y)), 10, (255, 0, 0), 2)  # blue
        cv2.imshow("debug Caver", debug_caver)

        # 计算目标图像的尺寸
        if hight is None or width is None:
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
        # translate_img = cv2.flip(translate_img, 1)  # 对角镜像
        # 返回矫正后的图像和成功标志
        return translate_img, True

    def crop_circle(self, img, center, radius):
        # 1. 获取图像高度、宽度
        h, w = img.shape[:2]

        # 2. 创建空白掩码（单通道，全黑）
        mask = np.zeros((h, w), dtype=np.uint8)

        # 3. 绘制白色填充圆形
        cv2.circle(mask, center, radius, 255, -1)

        # 4. 按位与操作，截取圆形区域
        circular_img = cv2.bitwise_and(img, img, mask=mask)

        # 显示结果（可选）
        # cv2.imshow("Original", img)
        # cv2.imshow("Circular Region", circular_img)
        return circular_img
