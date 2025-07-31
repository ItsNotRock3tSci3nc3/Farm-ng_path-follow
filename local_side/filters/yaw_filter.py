class YawFilter:
    def __init__(self, alpha=0.7):
        self.alpha = alpha
        self.filtered_yaw = None

    def update(self, raw_yaw, precision):
        '''
        Accuracy Value	Meaning	Action
        0.0 - 2.0	Good accuracy	Safe to trust
        2.0 - 5.0	Medium	Maybe OK, but be cautious
        > 5.0	Bad accuracy	Use filtered yaw or slow down
        > 10.0	Unreliable	Don't use for navigation
        '''

        #print(f"[YawFilter] Updating filtered_yaw: {self.filtered_yaw:.2f} from raw: {raw_yaw:.2f}, precision: {precision:.2f}")
        if precision > 5.0:
            return self.filtered_yaw or 0.0
        if self.filtered_yaw is None:
            self.filtered_yaw = raw_yaw
        else:
            self.filtered_yaw = self.alpha * self.filtered_yaw + (1 - self.alpha) * raw_yaw
        return self.filtered_yaw