"""盘点任务状态定义与强制记录的状态转移器。"""

from enum import Enum

from Lcode.ground_link import GroundMessageType


class InventoryState(str, Enum):
    BOOT = "BOOT"
    TARGET_SCAN = "TARGET_SCAN"
    WAIT_BUTTON = "WAIT_BUTTON"
    INIT_FLIGHT_HW = "INIT_FLIGHT_HW"
    PREFLIGHT = "PREFLIGHT"
    WARNING_5S = "WARNING_5S"
    TAKEOFF = "TAKEOFF"
    TRANSIT = "TRANSIT"
    SET_GIMBAL = "SET_GIMBAL"
    APPROACH_SLOT = "APPROACH_SLOT"
    VISUAL_ALIGN = "VISUAL_ALIGN"
    VISUAL_SERVO = "VISUAL_SERVO"
    VERIFY_QR = "VERIFY_QR"
    ILLUMINATE = "ILLUMINATE"
    REPORT = "REPORT"
    RETURN = "RETURN"
    LAND = "LAND"
    END = "END"
    FAULT = "FAULT"


ALLOWED_TRANSITIONS = {
    InventoryState.BOOT: {InventoryState.TARGET_SCAN, InventoryState.WAIT_BUTTON, InventoryState.FAULT},
    InventoryState.TARGET_SCAN: {InventoryState.WAIT_BUTTON, InventoryState.FAULT},
    InventoryState.WAIT_BUTTON: {InventoryState.INIT_FLIGHT_HW, InventoryState.FAULT},
    InventoryState.INIT_FLIGHT_HW: {InventoryState.PREFLIGHT, InventoryState.FAULT},
    InventoryState.PREFLIGHT: {InventoryState.WARNING_5S, InventoryState.FAULT},
    InventoryState.WARNING_5S: {InventoryState.TAKEOFF, InventoryState.FAULT},
    InventoryState.TAKEOFF: {InventoryState.TRANSIT, InventoryState.RETURN, InventoryState.FAULT},
    InventoryState.TRANSIT: {
        InventoryState.SET_GIMBAL,
        InventoryState.APPROACH_SLOT,
        InventoryState.RETURN,
        InventoryState.LAND,
        InventoryState.FAULT,
    },
    InventoryState.SET_GIMBAL: {InventoryState.APPROACH_SLOT, InventoryState.RETURN, InventoryState.FAULT},
    InventoryState.APPROACH_SLOT: {InventoryState.VISUAL_ALIGN, InventoryState.RETURN, InventoryState.FAULT},
    InventoryState.VISUAL_ALIGN: {InventoryState.VISUAL_SERVO, InventoryState.VERIFY_QR, InventoryState.TRANSIT, InventoryState.RETURN, InventoryState.FAULT},
    InventoryState.VISUAL_SERVO: {InventoryState.VERIFY_QR, InventoryState.RETURN, InventoryState.FAULT},
    InventoryState.VERIFY_QR: {
        InventoryState.VISUAL_ALIGN,
        InventoryState.ILLUMINATE,
        InventoryState.TRANSIT,          # 扫码失败→跳过该货位，下一格
        InventoryState.RETURN,
        InventoryState.FAULT,
    },
    InventoryState.ILLUMINATE: {InventoryState.REPORT, InventoryState.FAULT},
    InventoryState.REPORT: {
        InventoryState.TRANSIT,
        InventoryState.SET_GIMBAL,
        InventoryState.RETURN,
        InventoryState.FAULT,
    },
    InventoryState.RETURN: {InventoryState.LAND, InventoryState.FAULT},
    InventoryState.LAND: {InventoryState.END, InventoryState.FAULT},
    InventoryState.END: set(),
    InventoryState.FAULT: {InventoryState.RETURN, InventoryState.LAND, InventoryState.END},
}


class InventoryStateMachine:
    """状态切换的唯一入口，同时写关键日志并尝试广播。

    ground_link可以为空或不可用；广播结果永远不影响状态切换。
    """

    def __init__(self, trace, ground_link=None):
        self.trace = trace
        self.ground_link = ground_link
        self.state = InventoryState.BOOT
        self.trace.start(self.state.value)
        self._broadcast("boot")

    def _broadcast(self, reason, **fields):
        if self.ground_link is None:
            return None
        try:
            return self.ground_link.publish(
                GroundMessageType.STATE,
                {"state": self.state.value, "reason": reason, **fields},
            )
        except Exception as exc:
            self.trace.fault("ground_publish_failed", error=str(exc))
            return None

    def transition(self, new_state, reason, **fields):
        new_state = InventoryState(new_state)
        if new_state not in ALLOWED_TRANSITIONS[self.state]:
            raise ValueError(f"非法状态转移: {self.state.value} -> {new_state.value}")
        previous = self.state
        self.state = new_state
        self.trace.transition(
            new_state.value,
            reason,
            previous_state=previous.value,
            **fields,
        )
        self._broadcast(reason, previous=previous.value, **fields)

    def sample(self, **fields):
        return self.trace.sample(self.state.value, **fields)

    def fault(self, code, recover_to_return=True, **fields):
        self.trace.fault(code, **fields)
        if self.state not in {InventoryState.FAULT, InventoryState.END}:
            self.transition(InventoryState.FAULT, code, **fields)
        if recover_to_return and InventoryState.RETURN in ALLOWED_TRANSITIONS[self.state]:
            self.transition(InventoryState.RETURN, "fault_return", fault_code=code)
