"""简易日志模块 (不依赖 colorlog)"""
import logging
import os
import sys


def get_logger():
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    # 控制台
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter(
        "[%(levelname)s] %(asctime)s\n%(message)s", "%Y-%m-%d %H:%M:%S"
    ))
    logger.addHandler(console)

    # 文件
    try:
        path = os.path.dirname(os.path.realpath(sys.argv[0]))
        fh = logging.FileHandler(os.path.join(path, "fc_log.log"), encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            "[%(levelname)s] %(asctime)s\n%(message)s", "%Y-%m-%d %H:%M:%S"
        ))
        logger.addHandler(fh)
    except Exception:
        pass

    return logger


logger = get_logger()
