"""任务二新增联调阶段的遥测映射测试。"""

from shared.competition_2026_d_protocol import UavPhase

from drone_control.competition_2026_d.task2_mission import Task2Phase
from drone_control.competition_2026_d.task2_telemetry import (
    _PHASE_MAP,
    Task2TelemetryPublisher,
    Task2TelemetrySample,
)


def test_stationary_retakeoff_hover_maps_to_uav_hover():
    sample = Task2TelemetrySample(
        phase=Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF,
        base_state="NAVIGATE",
        position_xyz_m=(0.0, 0.5, 0.8),
        mission_success=True,
    )

    assert Task2TelemetryPublisher._phase(sample) == UavPhase.HOVER


def test_every_outer_task2_phase_has_telemetry_mapping():
    assert set(_PHASE_MAP) | {Task2Phase.DYNAMIC_LANDING} == set(Task2Phase)
