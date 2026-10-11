import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.actuators import warn_led, drop_bag


class TestWarnLed:
    def test_returns_true_placeholder(self, caplog):
        result = warn_led()
        assert result is True


class TestDropBag:
    def test_returns_true_placeholder(self, caplog):
        result = drop_bag()
        assert result is True
