# -*- coding: utf-8 -*-
import os
import sys
from pathlib import Path
import logging
import colorlog


def get_root():
    file = Path(__file__).resolve()
    root = file.parents[0]
    if str(root) not in sys.path:
        sys.path.append(str(root))
    root = Path(os.path.relpath(root, Path.cwd()))
    root = str(root) + '/'
    return root


def create_logger(mode):
    # 创建一个日志器
    logger = logging.getLogger("Robot")
    if not logger.handlers:
        # console输出
        if mode == "console":
            handler = logging.StreamHandler()
            logger.setLevel(logging.DEBUG)
            formatter = colorlog.ColoredFormatter(
                fmt='%(log_color)s[%(asctime)s.%(msecs)03d] %(filename)s -> %(funcName)s line:%(lineno)d [%(levelname)s] : %(message)s',
                datefmt='%Y-%m-%d  %H:%M:%S',
                log_colors={
                    'DEBUG': 'white',
                    'INFO': 'green',
                    'WARNING': 'yellow',
                    'ERROR': 'red',
                    'CRITICAL': 'bold_red',
                }
            )

        # exe下输出文件
        elif mode == "file":
            log_file = ROOT + "../../Robot.log"
            handler = logging.FileHandler(log_file, mode="w", encoding="UTF-8")
            logger.setLevel(logging.WARNING)
            formatter = logging.Formatter(
                fmt='[%(asctime)s.%(msecs)03d] %(filename)s -> %(funcName)s line:%(lineno)d [%(levelname)s] : %(message)s',
                datefmt='%Y-%m-%d  %H:%M:%S')

        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


ROOT = get_root()
IMAGE_DATA_PATH = ROOT + "../../image_data/"  # 数据集文件相对于robot的路径
PARAMS_YAML = ROOT + "../../params.yaml"  # params.yaml文件相对于robot的路径
SERVER_ADDR = ('localhost', 8000)  # 机械臂IP地址与端口号

LOG = create_logger("console")
# LOG = create_logger("file")
