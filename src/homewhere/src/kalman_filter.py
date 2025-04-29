import numpy as np

class KalmanFilter:
    def __init__(self, R, Q, dt=0.1, initial_dl=0.0, initial_rpm=0.0):
        self.x = np.array([[initial_dl], [initial_rpm]])
        self.P = np.eye(2)

        self.F = np.array([[1, dt * 0.0695 * 2 * np.pi / 60],
                           [0, 1]])
        self.B = np.array([[0.005], [0.1]])

        # New H matrix for 8 measurements
        self.H = np.array([
            [1, 0],
            [1, 0],
            [1, 0],
            [1, 0],
            [0, 1],
            [0, 1],
            [0, 1],
            [0, 1]
        ])

        self.R = R  
        self.Q = Q

    def predict(self, u=np.array([[0]])):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        return self.x.flatten()
