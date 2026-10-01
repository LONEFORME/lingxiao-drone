"""ResourceMonitor单元测试。

运行（先确保已 pip install pytest psutil）：
    cd drone_control/fire_patrol && python -m pytest test_resource_monitor.py -v
"""
import io
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.resource_monitor import ResourceMonitor


class FakeTemps:
    def __init__(self, current):
        self.current = current


class FakeProc:
    def __init__(self, cpu_pct=41.0, rss_mb=210.3):
        self._cpu_pct = cpu_pct
        self._rss_bytes = rss_mb * 1024 * 1024

    def cpu_percent(self, interval=None):
        return self._cpu_pct

    def memory_info(self):
        class Mem:
            pass
        mem = Mem()
        mem.rss = self._rss_bytes
        return mem


class FakeVirtualMemory:
    def __init__(self, percent=55.2, used_mb=890.5):
        self.percent = percent
        self.used = used_mb * 1024 * 1024


class TestSampleOnce:
    def test_writes_expected_fields(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 62.3)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(
            rm_module.psutil, "sensors_temperatures",
            lambda: {"pvt": [FakeTemps(46.319), FakeTemps(44.707)]},
            raising=False,  # Windows开发机上psutil无此属性(Linux专属)，板载环境(ubuntu-pi)存在
        )

        monitor = ResourceMonitor()
        monitor._proc = FakeProc(cpu_pct=41.0, rss_mb=210.3)
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()

        lines = log_file.getvalue().strip().split("\n")
        assert len(lines) == 1
        entry = json.loads(lines[0])
        assert entry["event"] == "resource"
        assert entry["cpu_percent_sys"] == 62.3
        assert entry["cpu_percent_proc"] == 41.0
        assert entry["mem_percent_sys"] == 55.2
        assert entry["mem_used_mb_sys"] == pytest.approx(890.5, abs=0.1)
        assert entry["mem_rss_mb_proc"] == pytest.approx(210.3, abs=0.1)
        assert entry["cpu_temp_c"] == pytest.approx(46.3, abs=0.1)
        assert "t" in entry

    def test_missing_temp_sensor_gives_none(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 10.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {}, raising=False)

        monitor = ResourceMonitor()
        monitor._proc = FakeProc()
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()

        entry = json.loads(log_file.getvalue().strip())
        assert entry["cpu_temp_c"] is None

    def test_psutil_exception_does_not_raise(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        def boom(interval=None):
            raise RuntimeError("psutil炸了")

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", boom)

        monitor = ResourceMonitor()
        monitor._proc = FakeProc()
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()  # 不应该抛异常

        assert log_file.getvalue() == ""


class TestStartStop:
    def test_start_produces_samples_stop_halts_them(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module, "SAMPLE_INTERVAL_S", 0.02)
        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 5.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {}, raising=False)
        monkeypatch.setattr(
            rm_module.psutil, "Process",
            lambda pid: FakeProc(cpu_pct=5.0, rss_mb=100.0),
        )

        monitor = ResourceMonitor()
        log_file = io.StringIO()
        log_lock = threading.Lock()

        monitor.start(log_file, log_lock)
        time.sleep(0.1)  # 至少经过几个SAMPLE_INTERVAL_S=0.02周期
        monitor.stop()

        lines_after_stop = log_file.getvalue().strip().split("\n")
        assert len(lines_after_stop) >= 2
        for line in lines_after_stop:
            entry = json.loads(line)
            assert entry["event"] == "resource"

        count_right_after_stop = len(lines_after_stop)
        time.sleep(0.1)  # 再等一轮周期，确认线程真的停了，没有继续写
        lines_later = log_file.getvalue().strip().split("\n")
        assert len(lines_later) == count_right_after_stop


class TestMissionIntegration:
    def test_mission_start_stop_writes_resource_events(self, monkeypatch, tmp_path):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module, "SAMPLE_INTERVAL_S", 0.02)
        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 5.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {}, raising=False)

        from Mission_GPT import mission

        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)

        # start()内部逻辑较多(涉及T265置信度确认等待、input()人工确认等)，
        # 这里不整体调用start()，只单独驱动跟本测试相关的两行(打开文件+起监控)，
        # 复刻start()里第320行之后的行为，避免测试卡在input()等待用户输入上。
        log_path = tmp_path / "flight_data.jsonl"
        m._log_file = open(log_path, "a")
        m._resource_monitor.start(m._log_file, m._log_lock)

        time.sleep(0.1)

        m.stop_all()

        content = log_path.read_text()
        lines = [json.loads(line) for line in content.strip().split("\n") if line]
        resource_lines = [e for e in lines if e.get("event") == "resource"]
        assert len(resource_lines) >= 2
