import numpy as np

class LQRController:
    
    def __init__(self, A, B, Q, R):

        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = None
        self.X = None
        self.eigvals = None
        
    def solve_continuous_are(self):

        n = self.A.shape[0]
        R_inv = np.linalg.inv(self.R)
        R_inv_BT = R_inv @ self.B.T
        
        H = np.block([
            [self.A, -self.B @ R_inv_BT],
            [-self.Q, -self.A.T]
        ])
        
        eigvals, eigvecs = np.linalg.eig(H)
        
    
        idx = np.argsort(np.real(eigvals))
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]
        

        stable_eigvecs = eigvecs[:, :n]
        U1 = stable_eigvecs[:n, :]
        U2 = stable_eigvecs[n:, :]
        
   
        X = np.real(U2 @ np.linalg.inv(U1))
        X = (X + X.T) / 2  
        
        return X
    
    def compute_gain(self):

        self.X = self.solve_continuous_are()
        
  
        R_inv = np.linalg.inv(self.R)
        self.K = R_inv @ self.B.T @ self.X

        A_cl = self.A - self.B @ self.K
        self.eigvals = np.linalg.eigvals(A_cl)
        
        return self.K, self.X, self.eigvals
    
    def get_gain(self):
        if self.K is None:
            self.compute_gain()
        return self.K
    
    def control(self, state):
        if self.K is None:
            self.compute_gain()
        
        state = np.array(state).reshape(-1, 1)
        return -self.K @ state
    
    def is_stable(self):
        if self.eigvals is None:
            self.compute_gain()
        return np.all(np.real(self.eigvals) < 0)

g = 9.81
m = 0.264
l = 0.0335
r = 0.0215
max_velocity = 1.0
THROTTLE_GAIN = 1.0
A = np.array([[0, 1, 0, 0],
              [g/l, 0, 0, 0],
              [0, 0, 0, 1],
              [-g/m, 0, 0, 0]])

B = np.array([[0],
              [-1 / (m * (l ** 2))],
              [0],
              [1 / m]])

Q = np.diag([0.7, 0.0, 0.0, 2.5])
R = np.array([[0.5]])
lqr_controller = LQRController(A, B, Q, R)
K, X, eigvals = lqr_controller.compute_gain()

LQR_K = LQRController(A, B, Q, R)

K = LQR_K.get_gain()

print(f"LQR_K = [ {K[0][0]}, {K[0][1]}, {K[0][2]}, {K[0][3]}]")
