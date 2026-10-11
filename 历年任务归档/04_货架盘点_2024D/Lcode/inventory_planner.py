"""货架盘点巡航与货位路径规划（纯逻辑，不访问飞控硬件）。"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, List, Optional, Sequence

from Lcode.warehouse_model import FaceId, FlightPoint, Slot, WarehouseModel


class WaypointKind(str, Enum):
    TAKEOFF = "takeoff"
    TRANSIT = "transit"
    SET_GIMBAL = "set_gimbal"
    INSPECT = "inspect"
    LAND_APPROACH = "land_approach"
    LAND = "land"


@dataclass(frozen=True)
class MissionWaypoint:
    point: FlightPoint
    kind: WaypointKind
    face: Optional[FaceId] = None
    slot_label: Optional[str] = None
    gimbal_angle_deg: Optional[float] = None


def point_distance(a: FlightPoint, b: FlightPoint) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


class InventoryPlanner:
    """生成安全通道路径。

    只有线段跨过货架平面Y时才加入上下端绕行点。上下端通道的选择按XY路程代价
    比较，其中下端X=+0.30是完整实飞后确认的安全值。
    """

    SAFE_RETURN_CRUISE_Z_TOLERANCE_M = 0.05

    FULL_FACE_ORDER: Sequence[FaceId] = (
        FaceId.A,
        FaceId.B,
        FaceId.C,
        FaceId.D,
    )

    def __init__(self, model: WarehouseModel = None):
        self.model = model or WarehouseModel()

    def _crossed_shelf_planes(self, start_y: float, end_y: float) -> List[float]:
        low, high = sorted((start_y, end_y))
        return [y for y in self.model.shelf_plane_y if low < y < high]

    def _choose_bypass_x(self, start: FlightPoint, end: FlightPoint) -> float:
        c = self.model.config
        candidates = (c.lower_bypass_x_m, c.upper_bypass_x_m)
        return min(candidates, key=lambda x: abs(start.x - x) + abs(end.x - x))

    def _safe_transit(
        self,
        start: FlightPoint,
        end: FlightPoint,
        *,
        bypass_x: Optional[float] = None,
    ) -> List[MissionWaypoint]:
        cruise_z = self.model.config.cruise_z_m
        # 两端高度一致时，跨货架绕行保持当前高度；只有需要改变高度的
        # 过渡才使用巡航高度，避免每次绕行都先爬回1.40m。
        transit_z = start.z if math.isclose(start.z, end.z, abs_tol=1e-9) else cruise_z
        start_transit = FlightPoint(start.x, start.y, transit_z)
        end_transit = FlightPoint(end.x, end.y, transit_z)
        points: List[FlightPoint] = []
        if not math.isclose(start.z, transit_z, abs_tol=1e-9):
            points.append(start_transit)

        if self._crossed_shelf_planes(start.y, end.y):
            if bypass_x is None:
                bypass_x = self._choose_bypass_x(start_transit, end_transit)
            points.extend(
                (
                    FlightPoint(bypass_x, start.y, transit_z),
                    FlightPoint(bypass_x, end.y, transit_z),
                )
            )

        points.append(end_transit)
        result: List[MissionWaypoint] = []
        for point in points:
            if result and point == result[-1].point:
                continue
            result.append(MissionWaypoint(point, WaypointKind.TRANSIT))
        return result

    def _scan_slots(
        self,
        face_id: FaceId,
        start_from_lower_end: bool,
        start_with_top: bool,
    ) -> List[Slot]:
        """按行分组+S形路径：先扫完一排（从远到近），换行后反方向扫回来。"""
        slots = list(self.model.slots_for_face(face_id))
        z_levels = sorted({slot.point.z for slot in slots}, reverse=start_with_top)
        result: List[Slot] = []
        for row_index, z in enumerate(z_levels):
            row = [s for s in slots if math.isclose(s.point.z, z)]
            # 第一行按 start_from_lower_end 方向，第二行反向（S形）
            reverse_x = start_from_lower_end if row_index == 0 else not start_from_lower_end
            row.sort(key=lambda s: s.point.x, reverse=reverse_x)
            result.extend(row)
        return result

    def _append_face(
        self,
        route: List[MissionWaypoint],
        face_id: FaceId,
        start_from_lower_end: bool,
        start_with_top: bool,
        bypass_x: Optional[float] = None,
    ) -> None:
        face = self.model.faces[face_id]
        slots = self._scan_slots(face_id, start_from_lower_end, start_with_top)
        first = slots[0].point
        current = route[-1].point
        route.extend(self._safe_transit(current, first, bypass_x=bypass_x))
        route.append(
            MissionWaypoint(
                point=first,
                kind=WaypointKind.SET_GIMBAL,
                face=face_id,
                gimbal_angle_deg=face.gimbal_angle_deg,
            )
        )
        route.extend(
            MissionWaypoint(
                point=slot.point,
                kind=WaypointKind.INSPECT,
                face=face_id,
                slot_label=slot.label,
                gimbal_angle_deg=face.gimbal_angle_deg,
            )
            for slot in slots
        )

    def plan_full_inventory(self) -> List[MissionWaypoint]:
        route = [MissionWaypoint(self.model.takeoff, WaypointKind.TAKEOFF)]
        for index, face_id in enumerate(self.FULL_FACE_ORDER):
            # 2026-07-22现场确认：完整盘点A→B、C→D均固定走上端
            # X=-2.80通道；单面、目标货位和安全返航仍按距离自动选择。
            forced_bypass_x = (
                self.model.config.upper_bypass_x_m
                if face_id in {FaceId.B, FaceId.D}
                else None
            )
            self._append_face(
                route,
                face_id,
                # 两次跨货架都从上端X=-2.80进入，四个面统一从
                # X=-1.75端开始S形扫描，避免进入B/D后再横穿到X=-0.70。
                start_from_lower_end=False,
                start_with_top=(index % 2 == 0),
                bypass_x=forced_bypass_x,
            )

        route.extend(self._safe_transit(route[-1].point, self.model.landing_approach))
        route.append(MissionWaypoint(self.model.landing_approach, WaypointKind.LAND_APPROACH))
        route.append(MissionWaypoint(self.model.landing_final, WaypointKind.LAND))
        return self._deduplicate_adjacent(route)

    def plan_face(self, face_id: FaceId) -> List[MissionWaypoint]:
        """Plan inventory for one face only (6 slots), landing after completion."""
        route = [MissionWaypoint(self.model.takeoff, WaypointKind.TAKEOFF)]
        for index, face in enumerate(self.FULL_FACE_ORDER):
            if face == face_id:
                self._append_face(
                    route, face,
                    start_from_lower_end=False,
                    start_with_top=(index % 2 == 0),
                )
                break
        route.extend(self._safe_transit(route[-1].point, self.model.landing_approach))
        route.append(MissionWaypoint(self.model.landing_approach, WaypointKind.LAND_APPROACH))
        route.append(MissionWaypoint(self.model.landing_final, WaypointKind.LAND))
        return self._deduplicate_adjacent(route)

    def plan_safe_return(self, current: FlightPoint) -> List[MissionWaypoint]:
        """Plan collision-safe transit to landing approach without descending."""
        approach = self.model.landing_approach
        cruise_z = self.model.config.cruise_z_m
        # 飞控回传的激光高度在定点悬停时会有厘米级波动。已接近巡航高度时直接
        # 规划水平路径；低层货位仍先原地爬升，再进入货架外侧通道。
        if math.isclose(
            current.z,
            cruise_z,
            abs_tol=self.SAFE_RETURN_CRUISE_Z_TOLERANCE_M,
        ):
            start = FlightPoint(current.x, current.y, cruise_z)
        else:
            start = current
        transit = self._safe_transit(start, approach)
        route = [
            MissionWaypoint(waypoint.point, WaypointKind.TRANSIT)
            for waypoint in transit
            if waypoint.point != approach
        ]
        route.append(MissionWaypoint(approach, WaypointKind.LAND_APPROACH))
        return self._deduplicate_adjacent(route)

    def plan_target(self, slot_label: str) -> List[MissionWaypoint]:
        label = slot_label.strip().upper()
        if label not in self.model.slots:
            raise KeyError(f"未知货位: {slot_label}")
        slot = self.model.slots[label]
        face = self.model.faces[slot.face]
        route = [MissionWaypoint(self.model.takeoff, WaypointKind.TAKEOFF)]
        route.extend(self._safe_transit(route[-1].point, slot.point))
        route.append(
            MissionWaypoint(
                FlightPoint(slot.point.x, slot.point.y, self.model.config.cruise_z_m),
                WaypointKind.SET_GIMBAL,
                face=slot.face,
                gimbal_angle_deg=face.gimbal_angle_deg,
            )
        )
        route.append(
            MissionWaypoint(
                slot.point,
                WaypointKind.INSPECT,
                face=slot.face,
                slot_label=slot.label,
                gimbal_angle_deg=face.gimbal_angle_deg,
            )
        )
        route.extend(self._safe_transit(route[-1].point, self.model.landing_approach))
        route.append(MissionWaypoint(self.model.landing_approach, WaypointKind.LAND_APPROACH))
        route.append(MissionWaypoint(self.model.landing_final, WaypointKind.LAND))
        return self._deduplicate_adjacent(route)

    @staticmethod
    def _deduplicate_adjacent(route: Iterable[MissionWaypoint]) -> List[MissionWaypoint]:
        result: List[MissionWaypoint] = []
        for waypoint in route:
            if (
                result
                and waypoint.point == result[-1].point
                and waypoint.kind == result[-1].kind
            ):
                continue
            result.append(waypoint)
        return result

    def route_length_m(self, route: Sequence[MissionWaypoint]) -> float:
        return sum(point_distance(a.point, b.point) for a, b in zip(route, route[1:]))
