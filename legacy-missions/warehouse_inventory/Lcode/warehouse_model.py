"""2024立体货架盘点赛题的几何模型。

图纸坐标使用(u, v)：u沿5m长边，v沿4m宽边。飞行坐标以起飞点为原点，
+Y沿u，-X沿v，+Z向上。所有轴交换和符号只允许出现在WarehouseTransform中。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, Tuple


class FaceId(str, Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"


@dataclass(frozen=True)
class FlightPoint:
    x: float
    y: float
    z: float

    def as_list(self):
        return [self.x, self.y, self.z]


@dataclass(frozen=True)
class Slot:
    label: str
    face: FaceId
    number: int
    point: FlightPoint


@dataclass(frozen=True)
class Face:
    face: FaceId
    shelf_u_m: float
    scan_y_m: float
    look_y_sign: int
    gimbal_angle_deg: float


@dataclass(frozen=True)
class WarehouseConfig:
    # 图纸测量值；起飞点中心和二维码高度仍需现场复核。
    start_u_m: float = 0.75
    start_v_m: float = 0.75
    landing_u_m: float = 4.25
    landing_v_m: float = 3.25
    shelf_u_m: Tuple[float, float] = (1.50, 3.50)
    shelf_v_min_m: float = 1.00
    shelf_v_max_m: float = 3.00
    column_v_m: Tuple[float, float, float] = (1.45, 1.95, 2.50)
    top_qr_z_m: float = 1.25
    bottom_qr_z_m: float = 0.85
    camera_z_offset_m: float = 0.0
    scan_standoff_m: float = 0.70
    scan_back_y_offset_m: float = 0.20
    # 2026-07-22 D面两轮实飞现场校正：原扫描线偏外，累计向-Y收近0.20m。
    d_scan_y_adjust_m: float = -0.20
    cruise_z_m: float = 1.25
    landing_approach_z_m: float = 1.25
    landing_final_z_m: float = 0.20

    # 用户依据实际机体外廓确认的下端通道；上端值仍是图纸初值，待实物确认。
    lower_bypass_x_m: float = 0.30
    upper_bypass_x_m: float = -2.80

    # 云台原型当前使用0/180度。实际角度和回差之后标定。
    look_positive_y_angle_deg: float = 0.0
    look_negative_y_angle_deg: float = 180.0

    def __post_init__(self):
        if len(self.shelf_u_m) != 2:
            raise ValueError("必须配置两组货架")
        if len(self.column_v_m) != 3:
            raise ValueError("每个货架面必须有三列")
        if not self.shelf_v_min_m < min(self.column_v_m):
            raise ValueError("二维码列必须位于货架端点之间")
        if not max(self.column_v_m) < self.shelf_v_max_m:
            raise ValueError("二维码列必须位于货架端点之间")
        if self.scan_standoff_m <= 0:
            raise ValueError("扫描距离必须为正数")
        if self.lower_bypass_x_m <= self.flight_x(self.shelf_v_min_m):
            raise ValueError("下端通道必须位于货架下端外侧")
        if self.upper_bypass_x_m >= self.flight_x(self.shelf_v_max_m):
            raise ValueError("上端通道必须位于货架上端外侧")

    def flight_x(self, drawing_v_m: float) -> float:
        return -(drawing_v_m - self.start_v_m)

    def flight_y(self, drawing_u_m: float) -> float:
        return drawing_u_m - self.start_u_m

    def flight_z_for_qr(self, qr_center_z_m: float) -> float:
        return qr_center_z_m - self.camera_z_offset_m


class WarehouseModel:
    def __init__(self, config: WarehouseConfig = None):
        self.config = config or WarehouseConfig()
        self.faces: Dict[FaceId, Face] = self._build_faces()
        self.slots: Dict[str, Slot] = self._build_slots()

    @property
    def takeoff(self) -> FlightPoint:
        return FlightPoint(0.0, 0.0, self.config.cruise_z_m)

    @property
    def landing_approach(self) -> FlightPoint:
        c = self.config
        return FlightPoint(
            c.flight_x(c.landing_v_m),
            c.flight_y(c.landing_u_m),
            c.landing_approach_z_m,
        )

    @property
    def landing_final(self) -> FlightPoint:
        p = self.landing_approach
        return FlightPoint(p.x, p.y, self.config.landing_final_z_m)

    @property
    def shelf_plane_y(self) -> Tuple[float, float]:
        return tuple(self.config.flight_y(u) for u in self.config.shelf_u_m)

    def _build_faces(self) -> Dict[FaceId, Face]:
        c = self.config
        first, second = c.shelf_u_m
        return {
            FaceId.A: Face(
                FaceId.A,
                first,
                c.flight_y(first - c.scan_standoff_m),
                +1,
                c.look_positive_y_angle_deg,
            ),
            FaceId.B: Face(
                FaceId.B,
                first,
                c.flight_y(first + c.scan_standoff_m + c.scan_back_y_offset_m),
                -1,
                c.look_negative_y_angle_deg,
            ),
            FaceId.C: Face(
                FaceId.C,
                second,
                c.flight_y(second - c.scan_standoff_m),
                +1,
                c.look_positive_y_angle_deg,
            ),
            FaceId.D: Face(
                FaceId.D,
                second,
                c.flight_y(second + c.scan_standoff_m + c.scan_back_y_offset_m)
                + c.d_scan_y_adjust_m,
                -1,
                c.look_negative_y_angle_deg,
            ),
        }

    def _build_slots(self) -> Dict[str, Slot]:
        c = self.config
        slots: Dict[str, Slot] = {}
        # 从各货架面的正面看，A/C与B/D的左右方向在图纸v轴上相反。
        for face_id, face in self.faces.items():
            if face.look_y_sign > 0:
                visible_columns = tuple(reversed(c.column_v_m))
            else:
                visible_columns = c.column_v_m

            for column_index, drawing_v in enumerate(visible_columns, start=1):
                for row_offset, qr_z in ((0, c.top_qr_z_m), (3, c.bottom_qr_z_m)):
                    number = column_index + row_offset
                    label = f"{face_id.value}{number}"
                    slots[label] = Slot(
                        label=label,
                        face=face_id,
                        number=number,
                        point=FlightPoint(
                            c.flight_x(drawing_v),
                            face.scan_y_m,
                            c.flight_z_for_qr(qr_z),
                        ),
                    )
        return slots

    def slots_for_face(self, face_id: FaceId) -> Iterable[Slot]:
        return (self.slots[f"{face_id.value}{i}"] for i in range(1, 7))
