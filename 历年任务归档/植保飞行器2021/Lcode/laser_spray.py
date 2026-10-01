"""激光笔喷洒控制 — 包装 gpio_led，到达网格后闪烁模拟播撒。

激光笔连接在 GPIO LED 通道上，通过闪烁 1-3 次模拟播撒动作。
闪烁周期 1-2s（题目要求）。
"""
from __future__ import annotations

import time
from typing import Optional

try:
    from Lcode.gpio_led import set_rgb_led
    _HARDWARE_AVAILABLE = True
except ImportError:
    _HARDWARE_AVAILABLE = False


class LaserSpray:
    """激光笔喷洒控制器。

    Args:
        flash_duration_s: 每次闪烁亮的时间（秒）
        flash_interval_s: 两次闪烁间隔（秒）
    """

    def __init__(
        self,
        flash_duration_s: float = 0.5,
        flash_interval_s: float = 0.8,
    ):
        self._flash_duration = flash_duration_s
        self._flash_interval = flash_interval_s
        self._available = _HARDWARE_AVAILABLE and set_rgb_led("OFF")

    def spray(self, times: int = 2, grid_id: Optional[int] = None) -> bool:
        """在网格闪烁激光笔 times 次。

        Args:
            times: 闪烁次数（1-3 次，题目要求）
            grid_id: 可选，仅供日志记录

        Returns:
            True 表示闪烁完成
        """
        times = max(1, min(3, times))

        if self._available:
            for i in range(times):
                set_rgb_led("G")  # 激光笔亮（复用绿色LED通道）
                time.sleep(self._flash_duration)
                set_rgb_led("OFF")  # 激光笔灭
                if i < times - 1:
                    time.sleep(self._flash_interval)
        else:
            # 无硬件环境下模拟闪烁（用于桌面测试）
            grid_tag = f" grid={grid_id}" if grid_id else ""
            print(f"[LaserSpray] 闪烁 {times} 次{grid_tag}")

        return True

    def is_available(self) -> bool:
        return self._available
