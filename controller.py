class PidController:

    def __init__(self, kp, ki, kd, bias=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.bias = bias

        self.past_error = 0
        self.prev_error = 0

    def compute_control(self, sp, pv):
        error = sp - pv

        p = self.kp * error
        i = self.ki * self.past_error
        d = self.kd * (error - self.prev_error)

        self.prev_error = error
        self.past_error += error

        return p + i + d + self.bias
