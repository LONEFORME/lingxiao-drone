"""Full warehouse-inventory mission entry point."""

import os
import time
from dataclasses import replace
from pathlib import Path

import Lcode.Lprotocol
from Lcode.ground_link import BroadcastGroundLink
from Lcode.inventory_planner import InventoryPlanner, MissionWaypoint, WaypointKind
from Lcode.inventory_state import InventoryState, InventoryStateMachine
from Lcode.inventory_store import InventoryStore
from Lcode.inventory_controller import (
    CameraSource,
    InventoryFlightMission,
    InventoryMissionConfig,
    InventoryMissionCoordinator,
    require_async_scan_for_flight,
    default_inventory_config,
)
from Lcode.laser_pointer import LaserPointer
from Lcode.Logger import logger
from Lcode.qr_vision import (
    QRConsensus,
    QRConsensusConfig,
    QRMapping,
    QRDecoder,
    VisionDebugCapture,
)
from Lcode.sensor_gimbal import SensorGimbal
from Lcode.state_debug_logger import StateDebugConfig, StateTrace
from Lcode.global_variable import sp_side
from t265 import t265_class
from Lcode.warehouse_model import FaceId, FlightPoint


START_BUTTON_POLL_S = 0.05

# The state machine is now connected.  Actual launch still requires the
# physical button, T265 initialization, camera/gimbal/laser preflight, and the
# existing five-second red-light warning.
# Keep the physical entry point locked by default. For an explicitly authorized
# bench/flight test, the operator must opt in at process start; the physical
# button, T265 checks, and five-second red warning remain mandatory.
WAREHOUSE_MISSION_READY = os.getenv("DRONE_WAREHOUSE_MISSION_READY", "0") == "1"


def wait_for_start_button():
    """Initialize only the button and green LED before the physical press."""
    try:
        from Lcode.gpio_button import GpioButton
        from Lcode.gpio_led import set_rgb_led
    except Exception as exc:
        logger.error(f"起飞 GPIO 模块加载失败: {exc}")
        return False

    button = GpioButton()
    led_is_off = True
    try:
        if not button.start():
            logger.error("一键起飞按钮初始化失败")
            return False
        led_is_off = False
        if not set_rgb_led("G"):
            logger.error("一键起飞绿灯点亮失败")
            return False
        logger.info("绿灯常亮：请完成 T265 拔插，然后按下一键起飞按钮")
        while not button.was_pressed():
            time.sleep(START_BUTTON_POLL_S)
        logger.info("一键起飞按钮已按下，开始初始化完整盘点任务")
        if not set_rgb_led("OFF"):
            logger.error("按键确认后关闭绿灯失败")
            return False
        led_is_off = True
        return True
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.error(f"一键起飞按钮等待失败: {exc}")
        return False
    finally:
        if not led_is_off:
            try:
                set_rgb_led("OFF")
            except Exception:
                pass
        button.stop()


def _close_resources(camera, gimbal, laser, ground, trace):
    for resource in (camera, gimbal, laser):
        if resource is not None:
            try:
                resource.close()
            except Exception as exc:
                logger.warning(f"盘点硬件关闭失败: {exc}")
    if ground is not None:
        ground.close()
    if trace is not None:
        trace.close()


def _apply_scan_height_override(route):
    """Adjust only inspection heights for a controlled vision test."""
    raw = os.getenv("DRONE_INVENTORY_SCAN_Z", "").strip()
    if not raw:
        return route, None
    scan_z = float(raw)
    if not 0.5 <= scan_z <= 2.0:
        raise ValueError("DRONE_INVENTORY_SCAN_Z must be between 0.5 and 2.0 m")
    return [
        replace(waypoint, point=replace(waypoint.point, z=scan_z))
        if waypoint.kind.value == "inspect"
        else waypoint
        for waypoint in route
    ], scan_z


def _hover_qr_test_route():
    """Build a fixed-point QR test route; never enabled by default."""
    raw = os.getenv("DRONE_HOVER_QR_Z", "1.30").strip()
    hover_z = float(raw)
    if not 0.5 <= hover_z <= 2.0:
        raise ValueError("DRONE_HOVER_QR_Z must be between 0.5 and 2.0 m")
    origin = FlightPoint(0.0, 0.0, hover_z)
    landing = FlightPoint(0.0, 0.0, 0.20)
    return [
        MissionWaypoint(origin, WaypointKind.TAKEOFF),
        MissionWaypoint(origin, WaypointKind.INSPECT, slot_label="HOVER"),
        MissionWaypoint(origin, WaypointKind.LAND_APPROACH),
        MissionWaypoint(landing, WaypointKind.LAND),
    ]


def main():
    logger.info("=" * 40)
    logger.info("warehouse_inventory — 完整立体货架盘点控制器")
    logger.info("=" * 40)

    if not WAREHOUSE_MISSION_READY:
        logger.error("完整盘点状态机未通过代码验收，保持安全锁定")
        return

    base_dir = Path(__file__).resolve().parent
    config = default_inventory_config(base_dir)
    dry_run = os.getenv("DRONE_DRY_RUN", "0").strip().lower() in {"1", "true"}
    require_async_scan_for_flight(config, dry_run=dry_run)
    vision_debug = VisionDebugCapture(
        base_dir / os.getenv("DRONE_VISION_DEBUG_DIR", "vision_debug")
    )
    trace = StateTrace(
        path=base_dir / "inventory_state.jsonl",
        config=StateDebugConfig.from_env(),
    )
    ground = BroadcastGroundLink()
    ground.start()
    state_machine = InventoryStateMachine(trace, ground)
    state_machine.transition(InventoryState.WAIT_BUTTON, "full_inventory")

    camera = gimbal = laser = None
    serial_fc = mission1 = coordinator = None
    store = None
    try:
        if not wait_for_start_button():
            state_machine.fault("button_gate_failed", recover_to_return=False)
            return
        state_machine.transition(InventoryState.INIT_FLIGHT_HW, "button_pressed")

        camera = CameraSource(config)
        gimbal = SensorGimbal()
        laser = LaserPointer()
        if not camera.start():
            state_machine.fault("camera_init_failed", recover_to_return=False)
            return
        if not gimbal.start():
            state_machine.fault("gimbal_init_failed", recover_to_return=False)
            return
        if not laser.start():
            state_machine.fault("laser_init_failed", recover_to_return=False)
            return

        planner = InventoryPlanner()
        requested_slot = os.getenv("DRONE_INVENTORY_SLOT", "").strip().upper()
        requested_face = os.getenv("DRONE_INVENTORY_FACE", "").strip().upper()
        hover_qr_test = os.getenv("DRONE_HOVER_QR_TEST", "0").strip().lower() in {
            "1", "true", "yes", "on"
        }
        if hover_qr_test:
            route = _hover_qr_test_route()
            expected_slots = {"HOVER"}
            requested_slot = "HOVER"
        elif requested_face:
            face_id = FaceId(requested_face)
            route = planner.plan_face(face_id)
            expected_slots = {
                slot.label
                for slot in planner.model.slots.values()
                if slot.face == face_id
            }
        elif requested_slot:
            if requested_slot not in planner.model.slots:
                raise ValueError(f"unknown inventory test slot: {requested_slot}")
            route = planner.plan_target(requested_slot)
            expected_slots = {requested_slot}
        else:
            route = planner.plan_full_inventory()
            expected_slots = set(planner.model.slots)
        route, scan_z_override = _apply_scan_height_override(route)
        first_face = next(
            (
                waypoint.face
                for waypoint in route
                if waypoint.kind == WaypointKind.SET_GIMBAL and waypoint.face is not None
            ),
            None,
        )
        if first_face is not None:
            logger.info(f"起飞前预置云台: face={first_face.value}")
            if not gimbal.set_face(first_face):
                raise RuntimeError("起飞前云台预置失败")
        mapping = QRMapping(config.qr_mapping_file)
        decoder = QRDecoder(mapping, decode_profile=config.qr_decode_profile)
        consensus = QRConsensus(QRConsensusConfig.from_env())
        store = InventoryStore(expected_slots)
        state_machine.transition(
            InventoryState.PREFLIGHT,
            "hardware_ready",
            route_waypoints=len(route),
            inspect_slots=sum(waypoint.kind.value == "inspect" for waypoint in route),
            test_slot=requested_slot or None,
            scan_z_override=scan_z_override,
        )

        re_fc = [0] * 14
        se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
        realsense = t265_class()
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)

        coordinator = InventoryMissionCoordinator(
            route,
            state_machine,
            gimbal,
            laser,
            camera,
            decoder,
            consensus,
            store,
            ground,
            config,
            vision_debug=vision_debug,
            planner=planner,
        )
        mission1 = InventoryFlightMission(
            re_fc, se_fc, realsense, serial_fc, route, coordinator
        )
        mission1.start()
        while mission1.task_running:
            time.sleep(0.1)

        store.save(base_dir / "inventory_results.json")
        trace.summary(
            complete=store.is_complete(),
            result_count=len(store.by_slot),
            missing_slots=store.missing_slots(),
        )
        logger.info(
            f"盘点结束：{len(store.by_slot)}/{len(expected_slots)}，完整={store.is_complete()}"
        )
    except KeyboardInterrupt:
        logger.info("完整盘点收到用户中断")
        if mission1 is not None and mission1.task_running:
            mission1.emergency()
    except Exception as exc:
        logger.exception("完整盘点启动或运行失败")
        if state_machine.state not in {InventoryState.FAULT, InventoryState.END}:
            try:
                state_machine.fault("mission_exception", error=str(exc))
            except Exception:
                pass
        if mission1 is not None and mission1.task_running:
            mission1.emergency()
    finally:
        if store is not None:
            try:
                store.save(base_dir / "inventory_results.json")
            except Exception:
                pass
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()
        if coordinator is not None:
            coordinator.shutdown(join_timeout_s=2.0)
            camera = None
        _close_resources(camera, gimbal, laser, ground, trace)


if __name__ == "__main__":
    main()
