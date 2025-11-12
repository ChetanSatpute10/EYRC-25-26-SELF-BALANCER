# the class LQR_gain is used to calculate the k matrix for finding the output of the motors.... the same classes use the various functions and mathematical calculations to find the perfect k matrix
# the functions used in the class are used to calculate k matrix in the terms such as first it sets the or locks if the equation is linearly time invariant and then computes gain based on the gravity constant and mass of the bot with all the scene objects
# also the radius for the calculation is the wheel radius
# using the numpy lib because control library wasnt available in coppelissim 
# using the LQR control system beacuse this eliminates the manual tuning and a perfect stable bot can be achieved 

import numpy as np
class LQR_gain:     #Declared a self class for lqr gain matrix calculator
    def __init__(self, A, B, Q, R):

        self.A = A    
        self.B = B
        self.Q = Q
        self.R = R
        self.K = None
        self.X = None
        self.eigvals = None
    def solve_continuous_are(self):
        n = self.A.shape[0]     #to make sure the Riccati equation is linear time invariant
        R_inv = np.linalg.inv(self.R)
        R_inv_BT = R_inv @ self.B.T
        H = np.block([
            [self.A, -self.B @ R_inv_BT],
            [-self.Q, -self.A.T]
        ])
        eigvals, eigvecs = np.linalg.eig(H) #solve the invariant riccati equation using eigenvalues and vectors 
        idx = np.argsort(np.real(eigvals))
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]
        stable_eigvecs = eigvecs[:, :n]
        U1 = stable_eigvecs[:n, :]
        U2 = stable_eigvecs[n:, :]
        X = np.real(U2 @ np.linalg.inv(U1))
        X = (X + X.T) / 2  
        return X

    def compute_gain(self): #this solves the riccati equation and minimizes the cost function 
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

g = 9.81   #for calculating the k matrix the constants are derived from the mathematical modelling and analysis as in TASK 1A
m = 0.264
l = 0.0335
r = 0.0215
WHEEL_RADIUS = 0.0215  
WHEEL_BASE = 0.1       
#this is to calculate the A matrix in the state space equation x_dot = Ax + Bu 
#calculated the B matrix for input state variables
A = np.array([[0, 1, 0, 0],
              [g/l, 0, 0, 0],
              [0, 0, 0, 1],
              [-g/m, 0, 0, 0]])

B = np.array([[0],
              [-1 / (m * (l ** 2))],
              [0],
              [1 / m]])
#CALCULATES the Q and R to minimize the cost function J
Q = np.diag([0.7, 0.0, 0.0, 2.5])

R = np.array([[0.5]])

lqr_controller = LQR_gain(A, B, Q, R)

#instance for stroing the k matrix in K
K, X, eigvals = lqr_controller.compute_gain() 


#defined a new class for clamping the speed smooths and mathematically store the speedd
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
#i added  kalman filter library for filtering the error and proper system behaviour calculations
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

#define a custom made mathematics library it involved the taylor series and convergence 
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

#uses the base data of the bot for finding the perfect orientation use quaternions for avoiding gimbal lock 
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
#uses a transformation for proper stabilsation it first used the rotational matrix transformation after giving it yaw it started to drift because of incorrect local and global alignment
#this converts the global velocity into the respect of the body local frame
#so created two functiond actually to get the local linear velocity and angular velocities of the body wrt to the global frame to avoid orientation lock or mismatch between orientation
def transform_velocity_to_local_frame(velocity_global, yaw_angle):
    vx_global = velocity_global[0]
    vy_global = velocity_global[1]
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    vx_local = vx_global * cos_yaw - vy_global * sin_yaw
    vy_local = vx_global * sin_yaw + vy_global * cos_yaw
    return [vx_local, vy_local]
#this is based on the rotational theory because we need to take into account for the adjustment in the yaw that the body covers while rotating
#this function then conerts the angular velocity of the body wrt to thr global frame to angular velocity of the bot wrt to it own local frame.....
def transform_angular_velocity_to_local(ang_vel_global, yaw_angle):
    wx_global = ang_vel_global[0]
    wy_global = ang_vel_global[1]
    cos_yaw = cos(-yaw_angle)
    sin_yaw = sin(-yaw_angle)
    wx_local = wx_global * cos_yaw - wy_global * sin_yaw
    return wx_local


#all the initialisations of the variables are done in this function we have also introduced the velocity control function and the function to calculate the k matrix for lqr controller
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
#created the part of the code which filters out the noise or the data which is collected from all the wheel encoders and the the orientation data from the function euler_to_quaternion
#therefore three code parts which are the pitch rate, wheel velocity and the forward or the ooutput velocity of the bot this is done for smooth filtering and cleaning
    self.kf_pitch_rate = KalmanFilter1D(
        process_variance=0.00001,     
        measurement_variance=0.001    

    )
# for filtering the wheel odom data the wheels captured apllied the kalma 1D filter here
    self.kf_wheel_vel = KalmanFilter1D(
        process_variance=0.00001,     
        measurement_variance=0.0005   

    )
# for filtering the wheel velocity means producing a filtered output to the motors for more precise and smooth working for the bot
    self.kf_forward_vel = KalmanFilter1D(
        process_variance=0.00001,      
        measurement_variance=0.001
    )
# defined all the state variables required for the state space representation
    self.pitch_rate_filtered = 0.0
    self.wheel_vel_avg_filtered = 0.0
    self.forward_velocity_filtered = 0.0
    self.last_tilt_angle = 0.0
    self.forward_velocity_local = 0.0
    self.left_wheel_vel = 0.0
    self.right_wheel_vel = 0.0
    self.target_forward_velocity = 0.0
    self.target_angular_velocity = 0.0 

# these variables were defined for driving the bot in the forward,backward left and right with the added logic in the arm manipulator 
    self.forward_speed = 0.20   # declared a setpoint tweaked as need depending on the situation felt good around 0.20 gave good speed
    self.turn_rate = 2.4   #kept it straight between 2.4 to 3.6 gave me much more turning speed at the starting pickup point and at the dropping point almost turning differentially in no time

    self.drop_up_speed = 1.2 #also declared a setpoint for raising or lowering the arm because the it was breaking it just gave random value directly in the joint vel functio

    self.current_position = 0.0 #initialised to start from 0.0 setpoint it goes towards left gives negative value of the sensor reading and positive if right and vice versa

    self.last_time = sim.getSimulationTime() #this was the most crucial for controlling the bot via the keyboard because at each sensing time step it was resetting to some gibberish value so took time step dt at each time step

def sysCall_sensing(): 
    #this function we used simply for getting the orientation of the bot in the global frame  purely the sensing part beacuse we need a closed loop feedback which we will get by updating all these values at each time step....
    orientation_global = self.sim.getObjectOrientation(self.body_handle, -1)
    roll_global = orientation_global[0]  #for roll
    pitch_global = orientation_global[1] # for pitch
    self.yaw_angle = orientation_global[2] #for yaw angle

    # this returns an array of bodys linear and angular velocity with respect to the world or global frame...now this converts that global velocities wrt global frame into the velocities wrt to local frame
    lin_vel_global, ang_vel_global = self.sim.getObjectVelocity(self.body_handle)
    vel_local = transform_velocity_to_local_frame(lin_vel_global, self.yaw_angle)  # for linear velocities vx,vy and vz
    self.forward_velocity_local = vel_local[0] 
    pitch_rate_body = transform_angular_velocity_to_local(ang_vel_global, self.yaw_angle) #fort angular acceleration wx, wy and wz 

    cos_yaw = cos(self.yaw_angle)   #defined these two cos and sin values because we took the projection of the vector like there will be three vector for roll, pitch and yaw so i took projection of them in cos ans sin 
    sin_yaw = sin(self.yaw_angle)

    tilt_angle_body = atan2(

        sin(roll_global) * cos_yaw + sin(pitch_global) * sin_yaw,

        cos(roll_global) * cos(pitch_global)

    )  #this gives the tilt of the body wrt pitch and roll of the body which were global to the world frame....

    #this was to give the velocity of the both the motors at each time step because we need speed data at each time step to adjust spped output in the controller output 
    left_vel = self.sim.getJointVelocity(self.left_joint_handle) or 0.0
    right_vel = self.sim.getJointVelocity(self.right_joint_handle) or 0.0
    mean_wheel_rate = 0.5 * (left_vel + right_vel) #did this to make sure that both the wheels rotated with the same speed to avoid difference in the rotating speed of the both the wheels

    self.pitch_rate_filtered = self.kf_pitch_rate.update(pitch_rate_body) #gets filtered data of the pitch tilt
    self.wheel_vel_avg_filtered = self.kf_wheel_vel.update(mean_wheel_rate) #gives the distance or kind of encoder data filtered through kalman filter

    self.forward_velocity_filtered = self.kf_forward_vel.update(self.forward_velocity_local) #give the bots body velocty filtered through the kalman filter
    #the variables down will store all the tilt data of the body with all the encoder and velocity data of the body these will all update after the time stamp of 0.01 seconds
    self.last_tilt_angle = tilt_angle_body
    self.left_wheel_vel = left_vel
    self.right_wheel_vel = right_vel

def arm_interrupt():
    #declareda separate function because the arm manipulators and the wheel joint i was having trouble in giving the output to both of them at the same time because they were kind of mixing the output logic so we decided to created the function for arm interrupt so it worked separately and actuation worked differently
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
    #therefore to keep the actuation commands for arm and for drive very different we therefore separated both the functions from each other arm_interrupt and actuation
    global vel_control, K_MAT
    #we took the simulation time difference because the difference between the current time and the time at which last update took place will tell us position of the bot 
    #so current difference tell how much time is elapsed and how much the bot is moved from its starting point
    
    current_time = sim.getSimulationTime()
    dt = current_time - self.last_time
    if dt <= 0:
        dt = 0.0
    self.last_time = current_time

    #these are for driving the bot through the up,down,left, right keys

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
    #here we get the errors in the velocity and the position control using the filtered data of encoders and the velocity this was done with each time stamp being updated
    velocity_error = self.target_forward_velocity - (self.wheel_vel_avg_filtered * WHEEL_RADIUS)
    position_error = 0.0 - self.current_position
    # these are the control law which will be used in the LQR controller to calculate the value of u in u = -Kx to properly these are then multiplied with the gains from the lqr_gains to find optimal K matrix so to smooth the bot
    x1 = 0.0 - self.last_tilt_angle         
    x2 = -self.pitch_rate_filtered          
    x3 = position_error                        
    x4 = velocity_error                   
    lqr_control_signal = (K_MAT[0] * x1 + 
                          K_MAT[1] * x2 + 
                          K_MAT[2] * x3 + 
                          K_MAT[3] * x4)
    
    total_control = lqr_control_signal
    #idhar dhyaan se state variables dekhna kyuki yaha pe total control tera output hai ki bot kis tarike se react karega inn outputs sesince apne pass wheel hai toh dividing by wheel radius apne ko rotational speed dega wheels ki 
    wheel_cmd_base = -total_control / WHEEL_RADIUS 
    #aur isko aisa samjho ki tumhe rotate karna hai while maintaining stability of the bot so jitna change tere angular velocity mai aya uska compensate karne ke liye tumhe utna hi angular velocity ek wheel se add karna padega aur ek wheel se subtract karna padega
    #turn differential yaha par dikhata hai ki at each time step if agar yaw mai changes hote hai toh wo kitne se honge... 
    #aur jo left wheel aur right wheel hai left aure light ke time pe turn differential will never be zero but yehi  cheez jab 2007 aur 2008 dabaya toh turn differential mai always zero store hoga..
    turn_differential = (self.target_angular_velocity * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    left_wheel_cmd = wheel_cmd_base + turn_differential
    right_wheel_cmd = wheel_cmd_base - turn_differential
    # ek fixed domain main motor ki speed lie karne ke liye instance create kiye taki velocity jyada na hojaye.... aur motor output limits ke within hi rahe
    left_wheel_cmd_final = vel_control.set_speed(left_wheel_cmd)
    right_wheel_cmd_final = vel_control.set_speed(right_wheel_cmd)
    arm_interrupt()
    # finally.... after all filtering and proper mathematical calculation final motor velocities agayi....
    self.sim.setJointTargetVelocity(self.left_joint_handle, left_wheel_cmd_final)
    self.sim.setJointTargetVelocity(self.right_joint_handle, right_wheel_cmd_final)

def sysCall_cleanup():
    if hasattr(self, 'sim') and hasattr(self, 'left_joint_handle'):
        self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
        self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
        print("Controller stopped")

#if the controller losses its orientation toh it will help regaining it from that much error or reinitialising the bot
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