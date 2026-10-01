"""盘点结果存储、编号/货位双向唯一性检查和本地持久化。"""

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Optional


@dataclass(frozen=True)
class InventoryResult:
    cargo_id: int
    slot_label: str
    confidence: float
    timestamp: float


class InventoryConflict(ValueError):
    pass


class InventoryStore:
    def __init__(self, expected_slots=None):
        self.expected_slots = set(expected_slots or [])
        self.by_slot: Dict[str, InventoryResult] = {}
        self.by_cargo: Dict[int, InventoryResult] = {}

    def add(
        self,
        cargo_id: int,
        slot_label: str,
        confidence: float,
        timestamp: Optional[float] = None,
    ) -> InventoryResult:
        cargo_id = int(cargo_id)
        slot_label = slot_label.strip().upper()
        confidence = float(confidence)
        if not 1 <= cargo_id <= 24:
            raise ValueError("货物编号必须在1~24")
        if self.expected_slots and slot_label not in self.expected_slots:
            raise ValueError(f"未知货位: {slot_label}")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("置信度必须在[0,1]")

        existing_slot = self.by_slot.get(slot_label)
        existing_cargo = self.by_cargo.get(cargo_id)
        if existing_slot and existing_slot.cargo_id != cargo_id:
            raise InventoryConflict(
                f"货位{slot_label}已记录编号{existing_slot.cargo_id}，不能改为{cargo_id}"
            )
        if existing_cargo and existing_cargo.slot_label != slot_label:
            raise InventoryConflict(
                f"编号{cargo_id}已位于{existing_cargo.slot_label}，不能重复到{slot_label}"
            )
        if existing_slot:
            return existing_slot

        result = InventoryResult(
            cargo_id,
            slot_label,
            confidence,
            time.time() if timestamp is None else float(timestamp),
        )
        self.by_slot[slot_label] = result
        self.by_cargo[cargo_id] = result
        return result

    def check_available(self, cargo_id: int, slot_label: str) -> None:
        """Validate uniqueness without persisting a result."""
        cargo_id = int(cargo_id)
        slot_label = slot_label.strip().upper()
        existing_slot = self.by_slot.get(slot_label)
        existing_cargo = self.by_cargo.get(cargo_id)
        if existing_slot and existing_slot.cargo_id != cargo_id:
            raise InventoryConflict(
                f"slot {slot_label} already contains cargo {existing_slot.cargo_id}"
            )
        if existing_cargo and existing_cargo.slot_label != slot_label:
            raise InventoryConflict(
                f"cargo {cargo_id} already belongs to slot {existing_cargo.slot_label}"
            )

    def query_cargo(self, cargo_id: int) -> Optional[InventoryResult]:
        return self.by_cargo.get(int(cargo_id))

    def missing_slots(self):
        return sorted(self.expected_slots - set(self.by_slot))

    def is_complete(self) -> bool:
        return bool(self.expected_slots) and not self.missing_slots() and len(self.by_cargo) == len(self.expected_slots)

    def save(self, path):
        output = {
            "results": [asdict(self.by_slot[key]) for key in sorted(self.by_slot)],
            "missing_slots": self.missing_slots(),
            "complete": self.is_complete(),
        }
        Path(path).write_text(
            json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
        )
