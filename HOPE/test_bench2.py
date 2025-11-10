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

# System parameters
g = 9.81
m = 0.264
l = 0.0335
r = 0.0215
max_velocity = 1.0
THROTTLE_GAIN = 1.0

# Robot physical parameters
WHEEL_RADIUS = 0.0215  # meters
WHEEL_BASE = 0.1       # meters (distance between wheels)

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

# Create LQR controller instance
lqr_controller = LQRController(A, B, Q, R)
K, X, eigvals = lqr_controller.compute_gain()

class contrained_vel:
    def __init__(self, min_vel, max_vel):
        self._cmd_vel = 0
        self.min_vel = min_vel
        self.max_vel = max_vel
    
    def get_speed(self):
        return self._cmd_vel
    
    def set_speed(self, value):
        if value > self.max_vel:
            self._cmd_vel = self.max_vel
        elif value < self.min_vel:
            self._cmd_vel = self.min_vel
        else:
            self._cmd_vel = value
        return self._cmd_vel

class KalmanFilter1D:
    """Simple 1D Kalman filter for single variable estimation"""
    def __init__(self, process_variance=0.00001, measurement_variance=0.001):
        self.estimate = 0.0
        self.error_covariance = 0.1
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
    
    def update(self, measurement):
        # Prediction step
        predicted_error_cov = self.error_covariance + self.process_variance
        
        # Update step
        kalman_gain = predicted_error_cov / (predicted_error_cov + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_covariance = (1 - kalman_gain) * predicted_error_cov
        
        return self.estimate

PI = 3.141592653589793

def sqrt(n):
    x = n 
    for _ in range(20):
        x = (x + n / x) / 2
    return x
    
def copysign(a, b):
    return abs(a) if b >= 0 else -abs(a)

def atan(z):
    return z / (1.0 + 0.28 * z * z)  

def atan2(y, x):
    if x > 0:
        return atan(y / x)
    elif x < 0 and y >= 0:
        return atan(y / x) + PI
    elif x < 0 and y < 0:
        return atan(y / x) - PI
    elif x == 0 and y > 0:
        return PI / 2
    elif x == 0 and y < 0:
        return -PI / 2
    else:
        return 0.0
    
def asin(x):
    if x < -1.0 or x > 1.0:
        return 0.0
    elif x == 1.0:
        return PI / 2
    elif x == -1.0:
        return -PI / 2
    else:
        return atan(x / sqrt(1.0 - x * x))

def cos(x):
    # Taylor series approximation for cosine
    result = 1.0
    term = 1.0
    for n in range(1, 10):
        term *= -x * x / ((2*n-1) * (2*n))
        result += term
    return result

def sin(x):
    # Taylor series approximation for sine
    result = x
    term = x
    for n in range(1, 10):
        term *= -x * x / ((2*n) * (2*n+1))
        result += term
    return result

def euler_from_quaternion(quat):
    x, y, z, w = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)  
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = copysign(PI / 2, sinp)
    else:
        pitch = asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

def transform_velocity_to_local_frame(velocity_global, yaw_angle):
    """Convert global velocity to robot's local frame using rotation"""
    vx_global = velocity_global[0]
    vy_global = velocity_global[1]
    
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    
    # Rotate velocity vector by -yaw to get local frame
    vx_local = vx_global * cos_yaw - vy_global * sin_yaw
    vy_local = vx_global * sin_yaw + vy_global * cos_yaw
    
    return [vx_local, vy_local]

def transform_angular_velocity_to_local(ang_vel_global, yaw_angle):
    """Convert global angular velocity to robot's local frame"""
    wx_global = ang_vel_global[0]
    wy_global = ang_vel_global[1]
    
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    
    # Rotate angular velocity to local frame
    wx_local = wx_global * cos_yaw - wy_global * sin_yaw
    
    return wx_local

def sysCall_init():
    sim = require('sim')
    
    global vel_control, K_MAT
    vel_control = contrained_vel(-180, 180)
    
    # FIX: Use the instance, not the class
    K_MAT = lqr_controller.get_gain()[0]  # Get the gain matrix (returns flattened array)
    
    self.sim = sim  # Store sim reference
    self.body_handle = sim.getObject('/body') 
    self.left_joint_handle = sim.getObject('/left_joint')  
    self.right_joint_handle = sim.getObject('/right_joint')  
    
    # State variables in LOCAL robot frame
    self.pitch_local = 0.0
    self.pitch_rate_local = 0.0
    self.pitch_rate_filtered = 0.0
    
    self.yaw_angle = 0.0
    
    # Velocity in robot's forward direction
    self.forward_velocity_local = 0.0
    self.forward_velocity_filtered = 0.0
    
    # Wheel feedback
    self.left_wheel_vel = 0.0
    self.right_wheel_vel = 0.0
    self.wheel_vel_avg_filtered = 0.0
    
    # Target velocity (commanded by user via keyboard)
    self.target_forward_velocity = 0.0
    self.target_angular_velocity = 0.0  # For turning
    
    # Keyboard control parameters
    self.forward_speed = 0.5  # m/s when moving forward
    self.turn_rate = 1.0      # rad/s when turning
    
    # Initialize Kalman filters for each variable
    self.kf_pitch_rate = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    self.kf_forward_vel = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    self.kf_wheel_vel = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    
    print("Bot initialized - Using LOCAL frame with keyboard control (Arrow keys)")
    print(f"K_MAT: {K_MAT}")
    
def sysCall_sensing(): 
    # Get orientation in GLOBAL frame
    orientation_global = self.sim.getObjectOrientation(self.body_handle, -1)
    roll_global = orientation_global[0]
    pitch_global = orientation_global[1]
    self.yaw_angle = orientation_global[2]
    
    # Get velocities in GLOBAL frame
    lin_vel_global, ang_vel_global = self.sim.getObjectVelocity(self.body_handle)
    
    # === CRITICAL: Transform to LOCAL robot frame ===
    # Convert linear velocity from global to local frame
    vel_local = transform_velocity_to_local_frame(lin_vel_global, self.yaw_angle)
    self.forward_velocity_local = vel_local[0]  # Forward velocity in robot's frame
    
    # Convert angular velocity from global to local frame
    self.pitch_rate_local = transform_angular_velocity_to_local(ang_vel_global, self.yaw_angle)
    
    # Calculate pitch in LOCAL frame (pitch around robot's X-axis)
    # This compensates for yaw rotation
    cos_yaw = cos(self.yaw_angle)
    sin_yaw = sin(self.yaw_angle)
    
    self.pitch_local = atan2(
        sin(roll_global) * cos_yaw + sin(pitch_global) * sin_yaw,
        cos(roll_global) * cos(pitch_global)
    )
    
    # Read wheel velocities
    self.left_wheel_vel = self.sim.getJointVelocity(self.left_joint_handle)
    self.right_wheel_vel = self.sim.getJointVelocity(self.right_joint_handle)

def sysCall_actuation():
    global vel_control, K_MAT
    
    ############### Keyboard Input ##############
    # Reset targets to zero (stop if no key is pressed)
    self.target_forward_velocity = 0.0
    self.target_angular_velocity = 0.0
    
    message, data, data2 = self.sim.getSimulatorMessage()
    if message == self.sim.message_keypress:
        if data[0] == 2007:  # Forward (up arrow)
            self.target_forward_velocity = self.forward_speed
            print("Moving Forward")
        elif data[0] == 2008:  # Backward (down arrow)
            self.target_forward_velocity = -self.forward_speed
            print("Moving Backward")
        elif data[0] == 2009:  # Left arrow key (turn left)
            self.target_angular_velocity = self.turn_rate
            print("Turning Left")
        elif data[0] == 2010:  # Right arrow key (turn right)
            self.target_angular_velocity = -self.turn_rate
            print("Turning Right")
    #########################################
    
    # Apply Kalman filtering to LOCAL frame measurements
    self.pitch_rate_filtered = self.kf_pitch_rate.update(self.pitch_rate_local)
    
    # Kalman filter forward velocity in local frame
    self.forward_velocity_filtered = self.kf_forward_vel.update(self.forward_velocity_local)
    
    # Average and Kalman filter wheel velocities
    avg_wheel_vel = (self.left_wheel_vel + self.right_wheel_vel) / 2.0
    self.wheel_vel_avg_filtered = self.kf_wheel_vel.update(avg_wheel_vel)
    
    # === VELOCITY CONTROL ===
    # Calculate velocity error in robot's LOCAL frame
    velocity_error = self.target_forward_velocity - (self.wheel_vel_avg_filtered * WHEEL_RADIUS)
    
    # === LQR CONTROL (operates in LOCAL frame) ===
    x1 = 0.0 - self.pitch_local         # Pitch error from upright
    x2 = -self.pitch_rate_filtered      # Pitch angular velocity
    x3 = 0.0                            # Unused state
    x4 = velocity_error                 # Forward velocity error
    
    # LQR feedback control
    lqr_control_signal = (K_MAT[0] * x1 + 
                          K_MAT[1] * x2 + 
                          K_MAT[2] * x3 + 
                          K_MAT[3] * x4)
    
    # Convert to wheel velocity command (base command for both wheels)
    wheel_cmd_base = -lqr_control_signal / WHEEL_RADIUS
    
    # === DIFFERENTIAL DRIVE for turning ===
    # Angular velocity produces differential wheel speeds
    # w = (v_right - v_left) / wheel_base
    # Rearranging: v_left = v_base - (wheel_base * w) / 2
    #              v_right = v_base + (wheel_base * w) / 2
    
    turn_differential = (self.target_angular_velocity * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    
    left_wheel_cmd = wheel_cmd_base - turn_differential
    right_wheel_cmd = wheel_cmd_base + turn_differential
    
    # Constrain output for each wheel
    left_wheel_cmd_final = vel_control.set_speed(left_wheel_cmd)
    right_wheel_cmd_final = vel_control.set_speed(right_wheel_cmd)
    
    # Apply to motors
    self.sim.setJointTargetVelocity(self.left_joint_handle, left_wheel_cmd_final)
    self.sim.setJointTargetVelocity(self.right_joint_handle, right_wheel_cmd_final)

def sysCall_cleanup():
    if hasattr(self, 'sim') and hasattr(self, 'left_joint_handle'):
        self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
        self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
        print("Controller stopped")

def reset_controller():
    self.kf_pitch_rate = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    self.kf_forward_vel = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    self.kf_wheel_vel = KalmanFilter1D(process_variance=0.00001, measurement_variance=0.001)
    
    self.pitch_rate_filtered = 0.0
    self.forward_velocity_filtered = 0.0
    self.wheel_vel_avg_filtered = 0.0
    self.target_forward_velocity = 0.0
    self.target_angular_velocity = 0.0
    
    if hasattr(self, 'sim') and hasattr(self, 'left_joint_handle'):
        self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
        self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
        print("Controller reset")