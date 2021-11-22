class PIDControl:
    def __init__(self, send_steering_callback):
        self.error = 0.0
        self.send_steering_callback = send_steering_callback

        self.prev_cte = 0
        self.int_cte = 0
        self.tau_p, self.tau_d, self.tau_i = 100, 20, 0.01

    def updateError(self, error):
        self.error = error
        self._updatePID()

    def _getError(self):
        return max(min(self.error, 100), -100)

    def _move(self, steering):
        self.send_steering_callback(steering)

    def _updatePID(self):
        cte = self._getError()
        diff_cte = cte - self.prev_cte
        self.prev_cte = cte
        self.int_cte += cte
        steer = -self.tau_p * cte - self.tau_d * diff_cte - self.tau_i * self.int_cte
        self._move(min(100, int(steer)))