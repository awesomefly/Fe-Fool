import cv2
import sys
import time
from image.image_find_focus import FocusFinder
from yolov5.detect_self import YoloDetecter
from windows.window_detection import DEFAULT_MODEL_PATH, yolo_to_pixel
from robot import robot_master, LOG, ROOT
from llm.multimodal_recognition_ark import MultimodalRecognizerArk


def diagnose_camera_issues():
    print("=== 相机诊断工具 ===")

    # 检查OpenCV版本
    print(f"OpenCV版本: {cv2.__version__}")

    # 尝试检测相机数量
    max_test = 4
    working_cameras = []

    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                name = f"Camera {i}"
                cv2.imshow(name, frame)
                cv2.waitKey(1)
                working_cameras.append(i)
                print(f"相机 {i}: 正常工作")

                auto_focus_value = cap.get(cv2.CAP_PROP_AUTOFOCUS)
                print(
                    f"相机 {i}: 自动对焦开关值为: {auto_focus_value} （1=开启, 0=关闭）"
                )

                focus_value = cap.get(cv2.CAP_PROP_FOCUS)
                print(f"相机 {i}: 对焦值为: {focus_value}")

                # 亮度
                # cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)

                # # 对比度
                # cap.set(cv2.CAP_PROP_CONTRAST, 32)

                # # 饱和度
                # cap.set(cv2.CAP_PROP_SATURATION, 64)

                # # 曝光
                # cap.set(cv2.CAP_PROP_EXPOSURE, -7)

            else:
                print(f"相机 {i}: 已连接但无法读取帧")
            cap.release()
        else:
            print(f"相机 {i}: 无法打开")

    print(f"\n找到 {len(working_cameras)} 个可用的相机: {working_cameras}")

    if not working_cameras:
        print("\n建议解决方案:")
        print("1. 检查相机物理连接")
        print("2. 安装/更新相机驱动程序")
        print("3. 关闭其他使用相机的应用程序")
        print("4. 尝试不同的USB端口")
        print("5. 重启计算机")


def get_camera_resolutions(camera_index=0):
    """
    通过尝试常见分辨率来获取摄像头支持的分辨率列表
    """
    cap = cv2.VideoCapture(camera_index)

    # 常见的分辨率列表（按从高到低排列）
    common_resolutions = [
        (2560, 1440),  # 2K
        (1920, 1080),  # Full HD
        (1280, 720),  # HD
        (1024, 768),  # XGA
        (800, 600),  # SVGA
        (640, 480),  # VGA
        (320, 240),  # QVGA
    ]

    supported_resolutions = []

    print("正在检测摄像头支持的分辨率...")
    print("-" * 50)

    for width, height in common_resolutions:
        # 尝试设置这个分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # 读取实际设置的分辨率
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 如果设置成功，就表示支持这个分辨率
        if actual_width == width and actual_height == height:
            fps = cap.get(cv2.CAP_PROP_FPS)
            supported_resolutions.append(
                {
                    "width": width,
                    "height": height,
                    "fps": fps,
                    "resolution": f"{width}x{height}",
                }
            )
            print(f"✓ {width}x{height} @ {fps:.2f} fps (支持)")
        else:
            print(f"✗ {width}x{height} (不支持)")

    cap.release()
    print("-" * 50)
    print(f"总共支持 {len(supported_resolutions)} 种分辨率\n")

    return supported_resolutions


def get_camera_info(camera_index=0):
    """
    获取摄像头的详细信息
    """
    cap = cv2.VideoCapture(camera_index)

    print("摄像头信息")
    print("-" * 50)

    # 获取各种属性
    properties = {
        "分辨率宽度": cv2.CAP_PROP_FRAME_WIDTH,
        "分辨率高度": cv2.CAP_PROP_FRAME_HEIGHT,
        "帧率 (FPS)": cv2.CAP_PROP_FPS,
        "亮度": cv2.CAP_PROP_BRIGHTNESS,
        "对比度": cv2.CAP_PROP_CONTRAST,
        "饱和度": cv2.CAP_PROP_SATURATION,
        "色调": cv2.CAP_PROP_HUE,
        "曝光": cv2.CAP_PROP_EXPOSURE,
        "自动对焦": cv2.CAP_PROP_AUTOFOCUS,
    }

    for prop_name, prop_id in properties.items():
        value = cap.get(prop_id)
        print(f"{prop_name}: {value}")

    cap.release()
    print("-" * 50 + "\n")


def test_camera_auto_focus(camera_index=0):
    """
    测试摄像头自动对焦功能
    """
    cap = cv2.VideoCapture(camera_index)

    # 尝试设置自动对焦
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    print("开启自动对焦")

    time.sleep(5)

    # 检查自动对焦是否支持
    v = cap.get(cv2.CAP_PROP_AUTOFOCUS)
    print(f"自动对焦值{v}")
    if not v:
        print("此摄像头不支持自动对焦")
        # cap.release()
        # return

    cap.set(cv2.CAP_PROP_FOCUS, 0.5)
    time.sleep(5)  # 等待对焦完成

    # 按q 退出
    while True:
        # 获取当前对焦值
        focus_value = cap.get(cv2.CAP_PROP_FOCUS)
        print(f"当前对焦值: {focus_value}")

        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Auto Focus Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()


def variance_of_laplacian(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    while True:
        ret, frame = cap.read()
        if not ret:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        focus_measure = cv2.Laplacian(gray, cv2.CV_64F).var()
        print(f"图像清晰度值: {focus_measure}")
        if focus_measure < 100:
            print("图像模糊，需要调整对焦！")
        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1000) & 0xFF
        if key == 27 or key == ord("q"):  # 按Esc或q退出
            break
    cap.release()
    cv2.destroyAllWindows()


def find_focus(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    focus_finder = FocusFinder()
    self_yolo = YoloDetecter(weights=DEFAULT_MODEL_PATH, device="cpu")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            return
        cv2.imshow("Camera", frame)
        # focus_image, res = focus_finder.find_focus(frame)
        focus_image, res = focus_finder.find_chesspiece(frame)
        if res:
            cv2.imshow("Camera Focus", focus_image)
            # focus_image = cv2.resize(focus_image, (640, 640))

            # res_img, yolo_list = self_yolo.detect(focus_image)
            # cv2.imshow("Yolo Detect", res_img)
        key = cv2.waitKey(1000) & 0xFF
        if key == 27 or key == ord("q"):  # 按Esc或q退出
            break
    cap.release()
    cv2.destroyAllWindows()


def find_chessboard():
    cap = cv2.VideoCapture(0)
    focus_finder = FocusFinder()
    while True:
        # frame = cv2.imread("/Users/bytedance/Downloads/IMG_5631.png")
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            return
        cv2.imshow("Camera", frame)
        chessboard_image, res = focus_finder.find_chessboard(frame)
        if chessboard_image is not None:
            cv2.imshow("Chessboard", chessboard_image)
        key = cv2.waitKey(1000) & 0xFF
        if key == 27 or key == ord("q"):  # 按Esc或q退出
            break


def detect():
    cap = cv2.VideoCapture(0)
    self_yolo = YoloDetecter(weights=DEFAULT_MODEL_PATH, device="cpu")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            return
        cv2.imshow("Camera", frame)
        res_img, yolo_list = self_yolo.detect(frame)
        if res_img is not None:
            cv2.imshow("Detect", res_img)
        key = cv2.waitKey(1000) & 0xFF
        if key == 27 or key == ord("q"):  # 按Esc或q退出
            break


def detect_and_play():
    imgs = [
        "/Users/bytedance/Downloads/IMG_5664.png",
        "/Users/bytedance/Downloads/IMG_5665.png",
    ]
    focus_finder = FocusFinder()
    self_yolo = YoloDetecter(weights=DEFAULT_MODEL_PATH, device="cpu")
    master = robot_master.ChessRobotMaster(think_depth=1)  # 象棋
    for img in imgs:
        # start_time = time.time()
        cur_img = cv2.imread(img)
        if cur_img is None:
            continue

        # 取最大轮廓，即棋盘
        focus_image, has_res = focus_finder.find_chessboard(cur_img)
        if has_res:
            cv2.imshow("Chessboard", focus_image)
            # key = cv2.waitKey(100000) & 0xFF
            # if key == 27 or key == ord("q"):  # 按Esc或q退出
            #     break

            res_img, yolo_list = self_yolo.detect(focus_image)
            # 将YOLO输出的归一化坐标转换为像素坐标
            pixel_list = yolo_to_pixel(yolo_list, res_img.shape[0], res_img.shape[1])

            # 按类型排序，如果相邻两帧的检测结果相同，则认为是可信的
            pixel_list.sort(key=lambda x: x[2], reverse=False)
            print(f"pixel_list:{pixel_list},{res_img.shape}")

            master.receive_message(
                topic="yolo_res", message=(pixel_list, res_img.shape)
            )


def ai_detect_and_play():
    imgs = [
        # "/Users/bytedance/Downloads/IMG_5664.png",
        "/Users/bytedance/Downloads/IMG_5655.png",
    ]

    llm_recognizer = MultimodalRecognizerArk(
        api_key="d5155a8e-50ea-43b3-9eaf-3abc6eba852a",
        endpoint="https://ark.cn-beijing.volces.com/api/v3/chat/completions",
    )

    master = robot_master.ChessRobotMaster(think_depth=1)  # 象棋
    for img in imgs:
        # start_time = time.time()
        cur_img = cv2.imread(img)
        if cur_img is None:
            continue

        result = llm_recognizer.detect_chess_pieces(
            cur_img, model="doubao-seed-2-0-lite-260215"
        )
        print(f"result:{result}")
        if result:
            master.receive_message(topic="llm_res", message=(result["pieces"],))


def play():
    master = robot_master.ChessRobotMaster(think_depth=1)  # 象棋

    shape1 = (2341, 2824, 3)
    pixel_list1 = [
        [1145.216562718153, 1810.689398765564, 8],
        [1170.0855128467083, 2594.7987484931946, 6],
        [1145.216562718153, 145.96496734023094, 13],
        [1126.5647803544998, 973.5019211769104, 15],
    ]
    shape2 = (2351, 2826, 3)
    pixel_list2 = [
        [1148.0466795265675, 1804.2644913196564, 8],
        [1176.7478622794151, 2589.1978254318237, 6],
        [1136.3998714983463, 149.0531681254506, 13],
        [1158.0297178924084, 1250.561020374298, 15],
    ]

    master.receive_message(topic="yolo_res", message=(pixel_list1, shape1))
    master.receive_message(topic="yolo_res", message=(pixel_list2, shape2))


if __name__ == "__main__":
    try:
        # diagnose_camera_issues()
        # get_camera_info()
        # get_camera_resolutions()
        # test_camera_auto_focus()
        # variance_of_laplacian()
        # find_focus()
        # find_chessboard()
        # detect_and_play()
        ai_detect_and_play()
        # play()
        # detect()
    except Exception as e:
        print(e)
        import traceback

        traceback.print_exc()
