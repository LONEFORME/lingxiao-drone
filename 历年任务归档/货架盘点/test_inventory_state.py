import io
import json

from Lcode.inventory_state import InventoryState, InventoryStateMachine
from Lcode.inventory_store import InventoryConflict, InventoryStore
from Lcode.state_debug_logger import StateDebugConfig, StateTrace
from Lcode.warehouse_model import WarehouseModel


class FakeGround:
    def __init__(self):
        self.messages = []

    def publish(self, message_type, payload):
        self.messages.append((message_type, payload))
        return len(self.messages) - 1


def _trace_records(stream):
    return [json.loads(line) for line in stream.getvalue().splitlines()]


def test_state_machine_forces_transition_logs_and_broadcasts():
    stream = io.StringIO()
    trace = StateTrace(stream=stream, config=StateDebugConfig(debug_enabled=True))
    ground = FakeGround()
    machine = InventoryStateMachine(trace, ground)
    machine.transition(InventoryState.WAIT_BUTTON, "full_inventory")
    machine.transition(InventoryState.INIT_FLIGHT_HW, "button_pressed")
    machine.sample(t265_confidence=3)

    records = _trace_records(stream)
    assert any(r["event"] == "state_exit" and r["state"] == "BOOT" for r in records)
    assert any(r["event"] == "state_enter" and r["state"] == "WAIT_BUTTON" for r in records)
    assert any(r["event"] == "state_sample" and r["state"] == "INIT_FLIGHT_HW" for r in records)
    assert ground.messages[-1][1]["state"] == "INIT_FLIGHT_HW"


def test_illegal_state_transition_is_rejected_before_logging():
    stream = io.StringIO()
    machine = InventoryStateMachine(
        StateTrace(stream=stream, config=StateDebugConfig(debug_enabled=False))
    )
    try:
        machine.transition(InventoryState.ILLUMINATE, "skip_all_safety")
    except ValueError as exc:
        assert "非法状态转移" in str(exc)
    else:
        raise AssertionError("跳过安全阶段的转移必须失败")
    assert machine.state == InventoryState.BOOT


def test_inventory_store_enforces_both_unique_directions(tmp_path):
    model = WarehouseModel()
    store = InventoryStore(model.slots)
    result = store.add(7, "A1", 0.9, timestamp=1.0)
    assert store.query_cargo(7) == result
    assert store.add(7, "A1", 0.8) == result

    try:
        store.add(7, "B1", 0.9)
    except InventoryConflict:
        pass
    else:
        raise AssertionError("同一编号不能出现在两个货位")

    try:
        store.add(8, "A1", 0.9)
    except InventoryConflict:
        pass
    else:
        raise AssertionError("同一货位不能对应两个编号")

    output = tmp_path / "inventory.json"
    store.save(output)
    data = json.loads(output.read_text(encoding="utf-8"))
    assert data["results"][0]["cargo_id"] == 7
    assert len(data["missing_slots"]) == 23
    assert data["complete"] is False
