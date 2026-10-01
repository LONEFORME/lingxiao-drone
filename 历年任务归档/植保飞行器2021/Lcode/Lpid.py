import simple_pid


class PID:
    def __init__(self, type=0, target=0, p=None, i=None, d=None) -> None:
        self.xyp = 0.7
        self.xyi = 0.002
        self.xyd = 0.05
        self.yawp = 1.5
        self.yawi = 0.0
        self.yawd = 0.3
        self.xylimit = 40
        self.yawlimit = 30

        if type == 0:
            kp = p if p is not None else self.xyp
            ki = i if i is not None else self.xyi
            kd = d if d is not None else self.xyd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.xylimit, self.xylimit)
        else:
            kp = p if p is not None else self.yawp
            ki = i if i is not None else self.yawi
            kd = d if d is not None else self.yawd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.yawlimit, self.yawlimit)

    def set_target(self, target):
        self.pid.setpoint = target

    def get_pid(self, current):
        return self.pid(current)

    def reset(self):
        self.pid.reset()
