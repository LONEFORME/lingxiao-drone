"""舵机投放执行器：默认锁定、单次动作、结果明确。"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Callable


class ActuatorState(enum.Enum):
    LOCKED = "locked"
    RELEASING = "releasing"
    RELEASED = "released"
    UNCERTAIN = "uncertain"


@dataclass(frozen=True)
class PayloadActuatorConfig:
    locked_command: float = 0.0
    release_command: float = 1.0
    feedback_timeout_s: float = 0.6


class PayloadActuator:
    """硬件写入和反馈读取通过窄接口注入，可接STM32或Pi PWM。"""

    def __init__(
        self,
        write_command: Callable[[float], None],
        released_feedback: Callable[[], bool | None],
        config: PayloadActuatorConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.write_command = write_command
        self.released_feedback = released_feedback
        self.config = config or PayloadActuatorConfig()
        self.clock = clock
        self.state = ActuatorState.LOCKED
        self._started = 0.0
        self.write_command(self.config.locked_command)

    def release_once(self) -> bool:
        if self.state != ActuatorState.LOCKED:
            return False
        self.write_command(self.config.release_command)
        self.state = ActuatorState.RELEASING
        self._started = self.clock()
        return True

    def poll(self) -> ActuatorState:
        if self.state != ActuatorState.RELEASING:
            return self.state
        feedback = self.released_feedback()
        if feedback is True:
            self.state = ActuatorState.RELEASED
        elif feedback is False and self.clock() - self._started >= self.config.feedback_timeout_s:
            self.state = ActuatorState.UNCERTAIN
        elif feedback is None and self.clock() - self._started >= self.config.feedback_timeout_s:
            self.state = ActuatorState.UNCERTAIN
        return self.state

    def safe_lock_before_flight(self) -> None:
        if self.state != ActuatorState.LOCKED:
            raise RuntimeError("任务中或投放结果不确定时禁止重新锁定舵机")
        self.write_command(self.config.locked_command)
