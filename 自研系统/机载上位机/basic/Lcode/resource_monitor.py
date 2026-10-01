"""板载CPU/内存/温度采样，独立后台线程周期性写入flight_data.jsonl。

circle_pole阶段2的视觉识别(pole_vision)/雷达监听(Lradar)/T265 SDK都是同一个
Python进程里的线程，本进程CPU%已经能回答"是circle_pole自己在吃CPU还是板子上
其他东西"，线程级CPU拆分收益不够抵消/proc读取+C扩展线程无法命名的复杂度，
本模块只做进程级+系统级采样。见
docs/superpowers/specs/2026-07-15-board-resource-monitor-design.md。
"""
import json
import os
import threading
import time

import psutil

SAMPLE_INTERVAL_S = 1.0


class ResourceMonitor:
    def __init__(self):
        self._thread = None
        self._stop_flag = threading.Event()
        self._log_file = None
        self._log_lock = None
        self._proc = None

    def start(self, log_file, log_lock):
        self._log_file = log_file
        self._log_lock = log_lock
        self._stop_flag.clear()
        # psutil.Process必须只创建一次并复用同一实例——cpu_percent()靠同一
        # 实例前后两次调用的时间差算百分比，每次新建实例读数会一直是0。
        self._proc = psutil.Process(os.getpid())
        # cpu_percent系接口首次调用是"热身"，返回值无意义，直接丢弃。
        psutil.cpu_percent(interval=None)
        self._proc.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag.set()
        if self._thread is not None:
            # stop_all()只在任务结束时调一次，不在实时控制路径上，阻塞至多
            # 2秒没有性能代价，但能保证stop()返回后线程真的不会再碰
            # _log_file，避免调用方紧接着close()文件时跟仍在跑最后一轮采样
            # 的线程产生"写已关闭文件"的竞态。
            self._thread.join(timeout=2.0)

    def _loop(self):
        while not self._stop_flag.is_set():
            self._sample_once()
            time.sleep(SAMPLE_INTERVAL_S)

    def _sample_once(self):
        try:
            cpu_temp_c = None
            temps = psutil.sensors_temperatures()
            if temps.get("pvt"):
                cpu_temp_c = temps["pvt"][0].current

            vm = psutil.virtual_memory()
            entry = {
                "event": "resource",
                "t": round(time.time(), 3),
                "cpu_percent_sys": psutil.cpu_percent(interval=None),
                "cpu_percent_proc": self._proc.cpu_percent(interval=None),
                "mem_percent_sys": vm.percent,
                "mem_used_mb_sys": round(vm.used / 1024 / 1024, 1),
                "mem_rss_mb_proc": round(self._proc.memory_info().rss / 1024 / 1024, 1),
                "cpu_temp_c": round(cpu_temp_c, 1) if cpu_temp_c is not None else None,
            }
            if self._log_file:
                with self._log_lock:
                    self._log_file.write(json.dumps(entry) + "\n")
                    self._log_file.flush()
        except Exception:
            pass
