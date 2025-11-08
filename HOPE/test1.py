LQR_K = [-2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827]   
WHEEL_RADIUS = 0.0215

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

def apply_exponential_smoothing(old_estimate, new_measurement, blend_factor=0.025):
    return old_estimate + blend_factor * (new_measurement - old_estimate)

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
    
    global vel_control
    vel_control = contrained_vel(-180, 180)

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
    
    # Target velocity (commanded by user)
    self.target_forward_velocity = 0.0
    
    # Filter coefficient
    self.ALPHA = 0.05
    
    self.sim = sim
    print("Bot initialized - Using LOCAL frame measurements")
    
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
    global vel_control
    
    # Apply smoothing to LOCAL frame measurements
    self.pitch_rate_filtered = apply_exponential_smoothing(
        self.pitch_rate_filtered, 
        self.pitch_rate_local, 
        self.ALPHA
    )
    
    # Smooth forward velocity in local frame
    self.forward_velocity_filtered = apply_exponential_smoothing(
        self.forward_velocity_filtered,
        self.forward_velocity_local,
        self.ALPHA
    )
    
    # Average and smooth wheel velocities
    avg_wheel_vel = (self.left_wheel_vel + self.right_wheel_vel) / 2.0
    self.wheel_vel_avg_filtered = apply_exponential_smoothing(
        self.wheel_vel_avg_filtered,
        avg_wheel_vel,
        self.ALPHA
    )
    
    # === VELOCITY CONTROL ===
    # Target is zero (no drift) - can be changed to commanded velocity later
    self.target_forward_velocity = 0.0
    
    # Calculate velocity error in robot's LOCAL frame
    velocity_error = self.target_forward_velocity - (self.wheel_vel_avg_filtered * WHEEL_RADIUS)
    
    # === LQR CONTROL (operates in LOCAL frame) ===
    x1 = 0.0 - self.pitch_local         # Pitch error from upright
    x2 = -self.pitch_rate_filtered      # Pitch angular velocity
    x3 = 0.0                            # Unused state
    x4 = velocity_error                 # Forward velocity error
    
    # LQR feedback control
    lqr_control_signal = (LQR_K[0] * x1 + 
                          LQR_K[1] * x2 + 
                          LQR_K[2] * x3 + 
                          LQR_K[3] * x4)
    
    # Convert to wheel velocity command
    wheel_cmd = -lqr_control_signal / WHEEL_RADIUS
    
    # Constrain output
    wheel_cmd_final = vel_control.set_speed(wheel_cmd)
    
    # Apply to both motors
    self.sim.setJointTargetVelocity(self.left_joint_handle, wheel_cmd_final)
    self.sim.setJointTargetVelocity(self.right_joint_handle, wheel_cmd_final)

def sysCall_cleanup():
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    print("Controller stopped")

def reset_controller():
    self.pitch_rate_filtered = 0.0
    self.forward_velocity_filtered = 0.0
    self.wheel_vel_avg_filtered = 0.0
    self.target_forward_velocity = 0.0
    
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    print("Controller reset")