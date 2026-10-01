"""状态机阶段日志；高频样本可关闭，关键状态转换和故障始终保留。"""

import json
import os
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from Lcode.Logger import logger


@dataclass(frozen=True)
class StateDebugConfig:
    debug_enabled: bool = False
    sample_interval_s: float = 0.10

    def __post_init__(self):
        if not 0.02 <= self.sample_interval_s <= 10.0:
            raise ValueError("状态调试采样周期必须在[0.02,10]秒内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        enabled = env.get("DRONE_STATE_DEBUG_LOG", "0").strip().lower()
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_STATE_DEBUG_LOG只能是0/1/false/true")
        return cls(
            debug_enabled=enabled in {"1", "true"},
            sample_interval_s=float(env.get("DRONE_STATE_DEBUG_INTERVAL_S", "0.10")),
        )


class StateTrace:
    def __init__(
        self,
        path: Optional[Path] = None,
        config: StateDebugConfig = None,
        stream=None,
        clock=time.monotonic,
        wall_clock=time.time,
    ):
        self.config = config or StateDebugConfig.from_env()
        self._clock = clock
        self._wall_clock = wall_clock
        self._lock = threading.Lock()
        self._stream = stream
        self._owns_stream = False
        if self._stream is None and path is not None:
            try:
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                self._stream = open(path, "a", encoding="utf-8", buffering=1)
                self._owns_stream = True
            except Exception as exc:
                logger.error(f"状态调试日志打开失败，控制逻辑继续: {exc}")
        self.current_state = None
        self._entered_at = None
        self._last_sample_by_state = {}

    def _emit(self, event: str, **fields) -> bool:
        if self._stream is None:
            return False
        record = {"t": round(self._wall_clock(), 3), "event": event, **fields}
        try:
            with self._lock:
                self._stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                self._stream.flush()
            return True
        except Exception as exc:
            logger.error(f"状态调试日志写入失败，控制逻辑继续: {exc}")
            return False

    def start(self, state: str, **fields):
        now = self._clock()
        self.current_state = state
        self._entered_at = now
        self._emit("state_enter", state=state, previous=None, **fields)

    def transition(self, new_state: str, reason: str, **fields):
        now = self._clock()
        previous = self.current_state
        duration = None if self._entered_at is None else round(now - self._entered_at, 3)
        if previous is not None:
            self._emit(
                "state_exit",
                state=previous,
                next_state=new_state,
                duration_s=duration,
                reason=reason,
                **fields,
            )
        self.current_state = new_state
        self._entered_at = now
        self._emit(
            "state_enter",
            state=new_state,
            previous=previous,
            reason=reason,
            **fields,
        )

    def sample(self, state: Optional[str] = None, **fields) -> bool:
        if not self.config.debug_enabled:
            return False
        active = state or self.current_state or "UNKNOWN"
        now = self._clock()
        last = self._last_sample_by_state.get(active)
        if last is not None and now - last < self.config.sample_interval_s:
            return False
        self._last_sample_by_state[active] = now
        return self._emit("state_sample", state=active, **fields)

    def fault(self, code: str, **fields):
        self._emit("fault", state=self.current_state, code=code, **fields)

    def summary(self, **fields):
        self._emit("mission_summary", state=self.current_state, **fields)

    def close(self):
        if self._owns_stream and self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
        self._stream = None
