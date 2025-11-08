LQR_K = [-2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827]   
WHEEL_RADIUS = 0.0215

class contrained_vel:
    def __init__(self, min_vel, max_vel):
        self._cmd_vel = 0
        self.min_vel = min_vel
        self.max_vel = max_vel
    
    def speed(self):
        return self._cmd_vel
    
    def speed(self, value):
        if value>self.max_vel:
            self._cmd_vel = self.max_vel
        elif value<self.min_vel:
            self._cmd_vel = self.min_vel
        else:
            self._cmd_vel = value

  

  

PI = 3.141592653589793
def sqrt (n):
    x = n 
    for _ in range (20):
        x = (x + n / x) / 2
        return x
    
def copysign(a,b):
    return abs(a) if b>=0 else -abs(a)

def atan(z):
    return z/(1.0 + 0.28 * z * z)  

def atan2(y,x):
    if x>0:
        return atan(y/x)
    elif x<0 and y>=0:
        return atan(y/x) + PI
    elif x<0 and y<0:
        return atan(y/x) - PI
    elif x==0 and y>0:
        return PI/2
    elif x==0 and y<0:
        return -PI/2
    else:
        return 0.0
    
def asin(x):
    if x<-1.0 or x>1.0:
        return 0.0
    elif x==1.0:
        return PI/2
    elif x==-1.0:
        return -PI/2
    else:
        return atan(x/sqrt(1.0 - x*x))

def euler_from_quaternion(quat):
    global roll, pitch, yaw
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
def sysCall_init():
    sim = require('sim')
    
    global vel_control, updated_vel
    vel_control = contrained_vel(-180,180)

    self.body_handle = sim.getObject('/body') 
    self.left_joint_handle = sim.getObject('/left_joint')  
    self.right_joint_handle = sim.getObject('/right_joint')  
    
    self.velocity_angular = 0.0
    self.velocity_linear_set_point = 0.0 
    self.yaw = 0.0  
    self.pitch_dot_filtered = 0.0
    self.velocity_angular_filtered = 0.0  
    
    self.SMOOTHING_COEFFICIENT = 0.025 
    
    self.pitch = 0.0
    self.pitch_dot = 0.0
    self.wheel_velocity = 0.0
    
    self.sim = sim
    
def sysCall_sensing(): 
    global vel_control, updated_vel
    quat = self.sim.getObjectQuaternion(self.body_handle, -1)
    euler = euler_from_quaternion(quat)
    
    self.pitch = euler[0]
    
    lin_vel, ang_vel = self.sim.getObjectVelocity(self.body_handle)
    self.pitch_dot = ang_vel[0] 
    self.wheel_velocity = updated_vel

    message,data,data2 = sim.getSimulatorMessage()

    if (message == sim.message_keypress):
        if (data[0]==2007): # forward up arrow
           forward_vel =  self.velocity_linear_set_point 

        elif (data[0]==2008): # backward down arrow
           backward_vel =  self.velocity_linear_set_point

        #  if (data[0]==2009): # left arrow key


        # if (data[0]==2010): # right arrow key

    else:
        forward_vel = 0.0
        backward_vel = 0.0

def sysCall_actuation():

    
    global vel_control, updated_vel
    self.pitch_dot_filtered = apply_exponential_smoothing(
        self.pitch_dot_filtered, 
        self.pitch_dot, 
        self.SMOOTHING_COEFFICIENT
    )
    
    self.velocity_angular_filtered = apply_exponential_smoothing(
        self.velocity_angular_filtered,
        self.wheel_velocity,
        self.SMOOTHING_COEFFICIENT
    )
    velocity_linear_error = self.velocity_linear_set_point - self.velocity_angular_filtered * WHEEL_RADIUS
    
    x1 = 0 - self.pitch 
    x2 = self.pitch_dot_filtered
    x3 = 0 
    x4 = velocity_linear_error
    
    lqr_v = LQR_K[0] * x1 + LQR_K[1] * x2 + LQR_K[2] * x3 + LQR_K[3] * x4
    
    wheel_vel = -lqr_v / WHEEL_RADIUS

    vel_control.speed = wheel_vel   
    updated_vel = vel_control.speed 

    self.sim.setJointTargetVelocity(self.left_joint_handle, updated_vel)
    self.sim.setJointTargetVelocity(self.right_joint_handle, updated_vel)

def sysCall_cleanup():
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    
    print("Self-Balancing Robot Controller Stopped")
   
def set_velocity_linear_set_point(vel):

    self.velocity_linear_set_point = vel
    print(f"Target velocity set to: {vel} m/s")

def set_yaw(yaw_rate):
    self.yaw = yaw_rate
    print(f"Yaw rate set to: {yaw_rate} rad/s")

def reset_controller():
    self.velocity_angular = 0.0
    self.velocity_linear_set_point = 0.0
    self.yaw = 0.0
    self.pitch_dot_filtered = 0.0
    self.velocity_angular_filtered = 0.0
    
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    
    print("Controller reset")