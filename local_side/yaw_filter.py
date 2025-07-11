class YawFilter:
    def __init__(self, alpha=0.7):
        self.alpha = alpha
        self.filtered_yaw = None

    def update(self, raw_yaw, precision):
        if precision <= 2:
            return self.filtered_yaw
        if self.filtered_yaw is None:
            self.filtered_yaw = raw_yaw
        else:
            self.filtered_yaw = self.alpha * self.filtered_yaw + (1 - self.alpha) * raw_yaw
        return self.filtered_yaw