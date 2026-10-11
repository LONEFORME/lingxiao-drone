"""
DMZ Protocol Test v2
====================
Design: AA/FF always lit, cls=FF+cnt=0=flight sentinel, cnt>=1=result
"""


class DmzTest:
    def __init__(self):
        self.se_dmz = [0xAA, 0, 0xFF, 0, 0xFF]  # Init: AA/FF always valid
        self.target_index = 0
        self.snapshot_log = []

    # --- Simulate navigate() update idx + reset sentinel ---
    def nav_update(self):
        self.se_dmz[1] = self.target_index & 0xFF
        if self.se_dmz[2] != 0xFF or self.se_dmz[3] != 0:
            self.se_dmz[2] = 0xFF
            self.se_dmz[3] = 0
        self.snapshot_log.append(list(self.se_dmz))

    # --- Simulate _detect_accept() write cls/cnt only ---
    def detect_accept(self, cls_id, count):
        self.se_dmz[2] = cls_id if cls_id < 5 else 0xFF
        self.se_dmz[3] = max(count, 1)
        self.snapshot_log.append(list(self.se_dmz))

    # --- Simulate _on_arrival() skip detected cell (no se_dmz change) ---
    def on_arrival_skip(self):
        pass

    def read_frame(self):
        return list(self.se_dmz)


class GroundStation:
    def __init__(self):
        self.last_idx = -1
        self.results = {}       # idx → (cls, cnt)
        self.progress = []      # idx per frame

    def receive(self, frame):
        aa, idx, cls, cnt, ff = frame
        self.progress.append(idx)

        if aa == 0xAA and ff == 0xFF:
            if idx != self.last_idx and cnt >= 1:
                self.last_idx = idx
                self.results[idx] = (cls, cnt)
                return f"DETECT: idx={idx} cls={cls} cnt={cnt}"
        return None


def log(msg, result):
    status = "OK" if result else "FAIL"
    print(f"  [{status}] {msg}")


def test_initial_state():
    """Init: AA/FF always lit"""
    t = DmzTest()
    f = t.read_frame()
    return f == [0xAA, 0, 0xFF, 0, 0xFF]


def test_nav_only_updates_idx():
    """navigate: only update idx+reset sentinel"""
    t = DmzTest()
    for i in range(5):
        t.target_index = i
        t.nav_update()
    f = t.read_frame()
    return f == [0xAA, 4, 0xFF, 0, 0xFF]


def test_detect_accept_writes_cls_cnt():
    """Detect writes cls/cnt only, AA/FF untouched"""
    t = DmzTest()
    t.target_index = 3
    t.nav_update()                 # Flight frame: [AA, 3, FF, 0, FF]
    t.detect_accept(cls_id=0, count=2)
    f = t.read_frame()
    return f == [0xAA, 3, 0, 2, 0xFF]  # cls=0 tiger, cnt=2


def test_nav_resets_after_detection():
    """Next frame: navigate resets cls=FF,cnt=0"""
    t = DmzTest()
    t.target_index = 3
    t.nav_update()
    t.detect_accept(cls_id=2, count=1)  # write monkey
    t.target_index = 4
    t.nav_update()                      # Reset
    f = t.read_frame()
    return f == [0xAA, 4, 0xFF, 0, 0xFF]


def test_on_arrival_skip_no_change():
    """Flyover detected cell: no se_dmz change"""
    t = DmzTest()
    t.target_index = 5
    t.nav_update()
    t.on_arrival_skip()
    t.target_index = 6
    t.nav_update()
    f = t.read_frame()
    return f == [0xAA, 6, 0xFF, 0, 0xFF]  # Flight state


def test_ground_station_dedup():
    """GS: only frames with AA/FF valid, cnt>=1 dedup"""
    gs = GroundStation()
    frames = [
        [0xAA, 0, 0xFF, 0, 0xFF],        # Flight
        [0xAA, 1, 0xFF, 0, 0xFF],        # Flight
        [0xAA, 2, 0,    2, 0xFF],        # point 2: tiger x2
        [0xAA, 2, 0,    2, 0xFF],        # Repeat (broadcast)
        [0xAA, 3, 0xFF, 0, 0xFF],        # Flying (nav reset)
        [0xAA, 4, 0xFF, 1, 0xFF],        # point 4: No animal
        [0xAA, 5, 0xFF, 0, 0xFF],        # Flight
        [0xAA, 6, 3,    1, 0xFF],        # point 6: peacock
        [0xAA, 7, 0xFF, 0, 0xFF],        # Flight
        [0xAA, 8, 3,    1, 0xFF],        # point 8: peacock (new idx->process)
        [0xAA, 8, 3,    1, 0xFF],        # repeat (dedup ignore)
    ]
    events = []
    for f in frames:
        r = gs.receive(f)
        if r:
            events.append(r)
    expected = [
        "DETECT: idx=2 cls=0 cnt=2",
        "DETECT: idx=4 cls=255 cnt=1",
        "DETECT: idx=6 cls=3 cnt=1",
        "DETECT: idx=8 cls=3 cnt=1",
    ]
    return events == expected and len(gs.results) == 4


def test_no_result_stick_to_flyover():
    """Detected cell flyover: cls=FF,cnt=0, GS ignores"""
    t = DmzTest()
    # Point 2 detected tiger
    t.target_index = 2
    t.nav_update()
    t.detect_accept(0, 2)
    # Flyover point 5
    t.target_index = 3
    t.nav_update()  # ResetSentinel
    t.target_index = 4
    t.nav_update()
    t.target_index = 5
    t.nav_update()
    t.on_arrival_skip()
    f = t.read_frame()
    # AA/FF intact, cls=FF,cnt=0 -> GS ignores
    return f == [0xAA, 5, 0xFF, 0, 0xFF]


def test_full_sequence():
    """Full 30 WP: 3 detection points + all idx tracked"""
    gs = GroundStation()
    t = DmzTest()
    animal_plan = {2: (0, 2), 7: (3, 1), 15: (0xFF, 1)}
    detected = set()

    for wp in range(30):
        t.target_index = wp
        t.nav_update()

        if wp in detected:
            t.on_arrival_skip()
            t.target_index = wp + 1
        elif wp in animal_plan:
            cls, cnt = animal_plan[wp]
            t.detect_accept(cls, cnt)
            detected.add(wp)
            t.target_index = wp + 1

    for f in t.snapshot_log:
        gs.receive(f)

    ok = len(gs.results) == len(animal_plan)
    for idx, (cls, cnt) in gs.results.items():
        if idx not in animal_plan:
            ok = False
        exp_cls = animal_plan[idx][0]
        expected = exp_cls if exp_cls < 5 else 0xFF
        if cls != expected:
            ok = False
    # Verifyall idx tracked
    ok = ok and len(gs.progress) >= 30
    return ok


if __name__ == "__main__":
    tests = [
        ("Init [AA,0,FF,0,FF]",         test_initial_state),
        ("navigate: only idx+ResetSentinel",        test_nav_only_updates_idx),
        ("Detect: writes cls/cnt",                test_detect_accept_writes_cls_cnt),
        ("Next frame nav resets sentinel",          test_nav_resets_after_detection),
        ("FlyoverSkipped cell unchanged",              test_on_arrival_skip_no_change),
        ("GSdedup(idx+cnt>=1)",         test_ground_station_dedup),
        ("Old result not sticky on flyover",              test_no_result_stick_to_flyover),
        ("Full 30-waypoint flow",                 test_full_sequence),
    ]

    all_pass = True
    for name, func in tests:
        try:
            ok = func()
            log(name, ok)
            if not ok:
                all_pass = False
        except Exception as e:
            log(name, False)
            print(f"        Exception: {e}")
            all_pass = False

    print()
    print(f"{'ALL PASSED' if all_pass else 'SOME FAILED'}")
