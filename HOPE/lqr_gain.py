import numpy as np

class calculating_lqr_gain:
    def __init__(self):
      self.lqr_gain = 0

    
A = np.array([[0, 1, 0, 0],
              [9.81/0.0335, 0, 0, 0],
              [0, 0, 0, 1],
              [-9.81/0.264, 0, 0, 0]])

B = np.array([[0],
              [-1 / (0.264 * (0.0335 ** 2))],
              [0],
              [1 / 0.264]])

Q = np.diag([0.7, 0.0, 0.0, 2.5]) 

R = np.array([[0.5]])

def solve_continuous_are(A, B, Q, R):
    n = A.shape[0] 
    R_inv = np.linalg.inv(R)
    R_inv_BT = R_inv @ B.T
    H = np.block([
        [A, -B @ R_inv_BT],
        [-Q, -A.T]
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

def lqr(A, B, Q, R):
    X = solve_continuous_are(A, B, Q, R)
    R_inv = np.linalg.inv(R)
    K = R_inv @ B.T @ X
    A_cl = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)
    
    return K, X, eigvals
K, X, eigvals = lqr(A, B, Q, R)





def connect_lqr_gain():
    lqr_gain_instance = calculating_lqr_gain()
    lqr_gain_instance.lqr_gain = K
    print(f"LQR Gain K: {lqr_gain_instance.lqr_gain}")