"""Test configuration and path bootstrapping for companion-computer."""

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASIC = ROOT / "basic"
SHARED = ROOT / "shared"
EDGE_VISION = ROOT.parent / "edge-vision"

for p in [ROOT, BASIC, SHARED, EDGE_VISION]:
    p_str = str(p)
    if p_str not in sys.path:
        sys.path.insert(0, p_str)

# Dynamic namespace alias: drone_control -> ROOT
if "drone_control" not in sys.modules:
    dc_mod = types.ModuleType("drone_control")
    dc_mod.__path__ = [str(ROOT)]
    sys.modules["drone_control"] = dc_mod

# Dynamic namespace alias: CyberCamera -> edge-vision
if "CyberCamera" not in sys.modules and EDGE_VISION.exists():
    cc_mod = types.ModuleType("CyberCamera")
    cc_mod.__path__ = [str(EDGE_VISION)]
    sys.modules["CyberCamera"] = cc_mod
