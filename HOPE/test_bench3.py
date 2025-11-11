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
WHEEL_RADIUS = 0.0215  
WHEEL_BASE = 0.1       

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

class contrained_vel:
    def __init__(self, min_vel, max_vel):
        self._cmd_vel = 0
        self._arm_speed = 0
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
    def __init__(self, process_variance=0.00001, measurement_variance=0.001):
        self.estimate = 0.0
        self.error_covariance = 0.1
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def update(self, measurement):
        predicted_error_cov = self.error_covariance + self.process_variance
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
    result = 1.0
    term = 1.0
    for n in range(1, 10):
        term *= -x * x / ((2*n-1) * (2*n))
        result += term
    return result

def sin(x):
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
    vx_global = velocity_global[0]
    vy_global = velocity_global[1]
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    vx_local = vx_global * cos_yaw - vy_global * sin_yaw
    vy_local = vx_global * sin_yaw + vy_global * cos_yaw
    return [vx_local, vy_local]

def transform_angular_velocity_to_local(ang_vel_global, yaw_angle):
    wx_global = ang_vel_global[0]
    wy_global = ang_vel_global[1]
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    wx_local = wx_global * cos_yaw - wy_global * sin_yaw
    return wx_local

def sysCall_init():

    sim = require('sim')
    global vel_control, K_MAT, dt
    vel_control = contrained_vel(-180, 180)
    K_MAT = lqr_controller.get_gain()[0] 
    self.sim = sim  
    self.body_handle = sim.getObject('/body') 
    self.left_joint_handle = sim.getObject('/left_joint')  
    self.right_joint_handle = sim.getObject('/right_joint')  
    self.arm_handle_joint = sim.getObject('/arm_joint')
    self.gripper_joint = sim.getObject('/Prismatic_joint')

    self.kf_pitch_rate = KalmanFilter1D(
        process_variance=0.00001,     
        measurement_variance=0.001    

    )

    self.kf_wheel_vel = KalmanFilter1D(
        process_variance=0.00001,     
        measurement_variance=0.0005   

    )

    self.kf_forward_vel = KalmanFilter1D(
        process_variance=0.00001,      
        measurement_variance=0.001
    )

    self.pitch_rate_filtered = 0.0
    self.wheel_vel_avg_filtered = 0.0
    self.forward_velocity_filtered = 0.0
    self.last_tilt_angle = 0.0
    self.forward_velocity_local = 0.0
    self.left_wheel_vel = 0.0
    self.right_wheel_vel = 0.0
    self.target_forward_velocity = 0.0
    self.target_angular_velocity = 0.0 

    self.forward_speed = 0.12   # for linear traversing as suitable   
    self.turn_rate = 1.5          

    self.drop_up_speed = 1.2
    self.grab_speed = 1.0

    self.current_position = 0.0

    self.last_time = sim.getSimulationTime()

    self.pitch_error_sum = 0.0

    self.windup_threshold = 0.01

    self.ki_coefficient = 0.0   

def sysCall_sensing(): 
    orientation_global = self.sim.getObjectOrientation(self.body_handle, -1)
    roll_global = orientation_global[0]
    pitch_global = orientation_global[1]
    self.yaw_angle = orientation_global[2]
    lin_vel_global, ang_vel_global = self.sim.getObjectVelocity(self.body_handle)
    vel_local = transform_velocity_to_local_frame(lin_vel_global, self.yaw_angle)
    self.forward_velocity_local = vel_local[0] 
    pitch_rate_body = transform_angular_velocity_to_local(ang_vel_global, self.yaw_angle)

    cos_yaw = cos(self.yaw_angle)
    sin_yaw = sin(self.yaw_angle)

    tilt_angle_body = atan2(

        sin(roll_global) * cos_yaw + sin(pitch_global) * sin_yaw,

        cos(roll_global) * cos(pitch_global)

    )
    left_vel = self.sim.getJointVelocity(self.left_joint_handle) or 0.0
    right_vel = self.sim.getJointVelocity(self.right_joint_handle) or 0.0
    mean_wheel_rate = 0.5 * (left_vel + right_vel)

    self.pitch_rate_filtered = self.kf_pitch_rate.update(pitch_rate_body)
    self.wheel_vel_avg_filtered = self.kf_wheel_vel.update(mean_wheel_rate)

    self.forward_velocity_filtered = self.kf_forward_vel.update(self.forward_velocity_local)

    current_pitch_error = -tilt_angle_body

    self.pitch_error_sum = max(-self.windup_threshold, 

                               min(self.windup_threshold, 

                                   self.pitch_error_sum + current_pitch_error))

    self.last_tilt_angle = tilt_angle_body
    self.left_wheel_vel = left_vel
    self.right_wheel_vel = right_vel

def arm_interrupt():
    self.target_arm_speed = 0.0
    message, data, data2 = self.sim.getSimulatorMessage()
    if message == self.sim.message_keypress:
        if data[0] == 119:  
            self.target_arm_speed = self.drop_up_speed
            print("lowering the arm")
        elif data[0] == 115: 
            self.target_arm_speed = -self.drop_up_speed
            print("lifting the arm")
        elif data[0] == 32:
            self.target_arm_speed = 0.0
            print("stopping the arm")
        self.sim.setJointTargetVelocity(self.arm_handle_joint, self.target_arm_speed)

def sysCall_actuation():
    global vel_control, K_MAT
    current_time = sim.getSimulationTime()
    dt = current_time - self.last_time
    if dt <= 0:
        dt = 0.0
    self.last_time = current_time
    message, data, data2 = self.sim.getSimulatorMessage()
    if message == self.sim.message_keypress:
        if data[0] == 2007:    
            self.target_forward_velocity = self.forward_speed
        elif data[0] == 2008: 
            self.target_forward_velocity = -self.forward_speed
        elif data[0] == 2009:  
            self.target_angular_velocity = self.turn_rate
        elif data[0] == 2010:  
            self.target_angular_velocity = -self.turn_rate
        elif data[0] == 32:  
            self.target_forward_velocity = 0.0
            self.target_angular_velocity = 0.0

    self.current_position += self.forward_velocity_filtered * dt
    velocity_error = self.target_forward_velocity - (self.wheel_vel_avg_filtered * WHEEL_RADIUS)
    position_error = 0.0 - self.current_position


    x1 = 0.0 - self.last_tilt_angle         
    x2 = -self.pitch_rate_filtered          
    x3 = position_error                        
    x4 = velocity_error                   
    lqr_control_signal = (K_MAT[0] * x1 + 
                          K_MAT[1] * x2 + 
                          K_MAT[2] * x3 + 
                          K_MAT[3] * x4)
    
    integral_comp = self.ki_coefficient * self.pitch_error_sum
    total_control = lqr_control_signal + integral_comp
    wheel_cmd_base = -total_control / WHEEL_RADIUS

    turn_differential = (self.target_angular_velocity * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    left_wheel_cmd = wheel_cmd_base + turn_differential
    right_wheel_cmd = wheel_cmd_base - turn_differential

    left_wheel_cmd_final = vel_control.set_speed(left_wheel_cmd)
    right_wheel_cmd_final = vel_control.set_speed(right_wheel_cmd)
    arm_interrupt()
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