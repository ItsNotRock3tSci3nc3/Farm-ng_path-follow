# Simple 2D Kalman filter for GPS
import numpy as np

class KalmanFilter2D:
    def __init__(self, process_var=1e-4, meas_var=1e-2):
        # State: [lat, lon]
        self.x = np.zeros((2, 1))
        self.P = np.eye(2)
        self.F = np.eye(2)
        self.H = np.eye(2)
        self.Q = np.eye(2) * process_var
        self.R = np.eye(2) * meas_var
        self.initialized = False

    def update(self, lat, lon):
        z = np.array([[lat], [lon]])
        if not self.initialized:
            self.x = z
            self.initialized = True
        # Prediction
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return float(self.x[0, 0]), float(self.x[1, 0])