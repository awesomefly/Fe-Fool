import cv2
import os
import sys
import time
import numpy as np
from windows.image_find_focus import FocusFinder


def image_show(frame, title="Image"):
    b, g, r, a = cv2.split(frame)
    rgb = cv2.merge([b, g, r])
    alpha = a / 255.0

    bg_color = (255, 255, 255)  # 白色背景
    bg = np.full(rgb.shape, bg_color, dtype=np.uint8)
    # 公式：out = rgb*alpha + bg*(1-alpha)
    out = (
        rgb.astype(float) * alpha[:, :, None]
        + bg.astype(float) * (1 - alpha[:, :, None])
    ).astype(np.uint8)

    cv2.imshow(title, out)


def find_focus():
    shape, hight, width = "rectangle", 1280, 1280
    input_path = "/Users/bytedance/Downloads/boardgame/background/"

    # shape, hight, width = "circle", 102, 102
    # input_path = "/Users/bytedance/Downloads/boardgame/foreground"
    if os.path.isfile(input_path) and input_path.endswith(".png"):
        frame = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
        if frame is None:
            print(f"Error: Could not read the image: {input_path}")
            return

        image_show(frame, "Raw with White BG")

        focus_finder = FocusFinder()
        focus_image, res = focus_finder.crop_image(
            frame, shape=shape, hight=hight, width=width
        )
        if res:
            image_show(focus_image, "Focus")  # todo：流冲突这里不更新
            dir = os.path.dirname(input_path)
            filename = os.path.basename(input_path)
            sep = filename.split(".")
            p_name = dir + "/" + sep[0] + "-" + str(int(time.time())) + ".png"
            print(f"dir: {dir}, file: {filename}, p_name: {p_name}")
            # cv2.imwrite(p_name, focus_image)
        key = cv2.waitKey(100000) & 0xFF
        if key == 27 or key == ord("q"):  # 按Esc或q退出
            pass
        return
    else:
        try:
            items = os.listdir(input_path)
            for name in items:
                path = os.path.join(input_path, name)
                if os.path.isdir(path) or not name.endswith(".png"):
                    print(f"DIR - {path}")
                    continue
                else:
                    print(f"FILE - {path}")
                    frame = cv2.imread(path, cv2.IMREAD_UNCHANGED)
                    if frame is None:
                        print(f"Error: Could not read the image: {path}")
                        return
                    cv2.imshow("Raw", frame)

                    focus_finder = FocusFinder()
                    focus_image, res = focus_finder.crop_image(
                        frame, shape=shape, hight=hight, width=width
                    )
                    if res:
                        cv2.imshow("Focus", focus_image)

                        sep = name.split(".")
                        p_name = (
                            input_path
                            + "/output/"
                            + sep[0]
                            + "-"
                            + str(int(time.time()))
                            + ".png"
                        )
                        print(f"dir: {path}, file: {name}, p_name: {p_name}")
                        cv2.imwrite(p_name, focus_image)

                    key = cv2.waitKey(100000) & 0xFF
                    if key == 27 or key == ord("q"):  # 按Esc或q退出
                        break
                    else:
                        pass
        except Exception as e:
            print(f"读取目录失败: {e}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    find_focus()
