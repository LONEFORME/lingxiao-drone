"""完整盘点航点的独立 route-only 实飞入口。

只执行 XYZ 航点，不执行二维码、云台和激光业务逻辑。正式盘点入口的
按键门禁保持不变：启动前必须输入 ROUTE_ONLY，并通过同一套按键门禁。
"""

import os
import time
from pathlib import Path

import Lcode.Lprotocol
from Lcode.global_variable import sp_side
from Lcode.Logger import logger
from Mission_GPT import mission
from main import wait_for_start_button
from t265 import t265_class


ROUTE_FILE = Path(__file__).with_name("router_full_inventory_test.txt")
ROUTE_CONFIRMATION = "ROUTE_ONLY"


def validate_route(path: Path):
    points = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 3:
            raise ValueError(f"航点第 {line_no} 行不是 XYZ 三元组")
        point = tuple(float(part) for part in parts)
        if not all(value == value for value in point):
            raise ValueError(f"航点第 {line_no} 行包含 NaN")
        if point[2] <= 0 or point[2] > 1.4:
            raise ValueError(f"航点第 {line_no} 行高度超出 (0, 1.4]：{point}")
        points.append(point)
    if len(points) != 40:
        raise ValueError(f"完整路线必须有 40 个航点，实际为 {len(points)}")
    if sum(abs(point[0] - 0.30) < 1e-9 for point in points) != 4:
        raise ValueError("完整路线必须包含 4 个 X=+0.30m 下端绕行点")
    if points[-1] != (-2.5, 3.5, 0.2):
        raise ValueError(f"降落终点不符合预期：{points[-1]}")
    return points


def configure_route_only_environment():
    """配置 route-only 的快速航段模式，供入口和单元测试共同使用。"""
    os.environ["DRONE_NAV_PROFILE"] = "cruise"
    os.environ["DRONE_CRUISE_CONFIRM_CYCLES"] = "2"
    os.environ["DRONE_CRUISE_REQUIRE_Z"] = "1"


def main():
    points = validate_route(ROUTE_FILE)
    logger.info(f"route-only 路线校验通过：{len(points)} 个航点，文件={ROUTE_FILE}")
    if input("输入 ROUTE_ONLY 才继续：").strip() != ROUTE_CONFIRMATION:
        logger.error("route-only 人工确认未通过，退出且不初始化飞控")
        return
    if not wait_for_start_button():
        logger.error("一键起飞门禁失败，退出且不解锁")
        return

    # 中间航点采用快速巡航确认，但仍要求高度到位，不能因 XY 到位就
    # 跳过上下层切换。首尾航点仍由 Mission_GPT 保持 precision。
    configure_route_only_environment()

    re_fc = [0] * 14
    se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
    realsense = t265_class()
    serial_fc = None
    mission1 = None
    try:
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)
        mission1 = mission(
            re_fc, se_fc, realsense, serial_fc, route_file=str(ROUTE_FILE)
        )
        mission1.start()
        while mission1.task_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("route-only 收到用户中断")
        if mission1 is not None:
            mission1.emergency()
            if not mission1.task_running:
                mission1.stop_all()
    finally:
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()


if __name__ == "__main__":
    main()
