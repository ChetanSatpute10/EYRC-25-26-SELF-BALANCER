"""
Self-Balancing Robot LQR Controller for CoppeliaSim
Converted from MuJoCo implementation
Configured for Task_1B scene with body, right_joint, and left_joint
"""

import math

# LQR gains obtained from calculate_lqr_gains.py
LQR_K = [-2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827]
WHEEL_RADIUS = 0.0215
MAX_MOTOR_VEL = 500.0  # rad/s

def clamp(n, minn, maxn):
    """Clamp value between min and max"""
    return max(min(maxn, n), minn)

def euler_from_quaternion(quat):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    quat: [x, y, z, w] format
    Returns: [roll, pitch, yaw] in radians
    """
    x, y, z, w = quat
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]


def sysCall_init():
    sim = require('sim')
    
    ######## INITIALIZATION #######
    # Get object handles from your scene
    self.body_handle = sim.getObject('/body')  # Main robot body
    self.left_joint_handle = sim.getObject('/left_joint')  # Left wheel joint
    self.right_joint_handle = sim.getObject('/right_joint')  # Right wheel joint
    
    # Controller state variables
    self.velocity_angular = 0.0
    self.velocity_linear_set_point = 0.0  # Target linear velocity (m/s)
    self.yaw = 0.0  # Yaw control for turning
    self.pitch_dot_filtered = 0.0  # Filtered pitch angular velocity
    self.velocity_angular_filtered = 0.0  # Filtered wheel velocity
    
    # State feedback variables (updated in sensing)
    self.pitch = 0.0
    self.pitch_dot = 0.0
    self.wheel_velocity = 0.0
    
    # Store sim reference
    self.sim = sim
    
    print("Self-Balancing Robot LQR Controller Initialized")
    print("LQR Gains: K =", LQR_K)
    print("Wheel Radius:", WHEEL_RADIUS, "m")
    print("Max Motor Velocity:", MAX_MOTOR_VEL, "rad/s")
    ##################################


def sysCall_sensing():
    ######## SENSING & ERROR CALCULATION ######
    
    # 1. Get pitch angle (tilt of robot body)
    # Get orientation quaternion [x, y, z, w]
    quat = self.sim.getObjectQuaternion(self.body_handle, -1)
    euler = euler_from_quaternion(quat)
    
    # Pitch is the roll angle (rotation about x-axis)
    # Negative sign to match MuJoCo convention
    self.pitch = euler[0]
    
    # 2. Get pitch angular velocity (rate of tilting)
    # getObjectVelocity returns [linear_velocity, angular_velocity]
    lin_vel, ang_vel = self.sim.getObjectVelocity(self.body_handle)
    self.pitch_dot = ang_vel[0]  # Angular velocity about x-axis
    
    # 3. Get wheel velocities
    vel_left = self.sim.getJointVelocity(self.left_joint_handle)
    vel_right = self.sim.getJointVelocity(self.right_joint_handle)
    
    # Average wheel velocity
    # Note: One wheel may be rotated 180deg, so we need to account for that
    # If wheels rotate in opposite directions for forward motion:
    self.wheel_velocity = (vel_left + vel_right) / 2.0
    
    # Alternative if both wheels rotate in same direction:
    # self.wheel_velocity = (vel_left + vel_right) / 2.0
    
    ###########################################


def sysCall_actuation():
    ######## LQR CONTROL ACTUATION ######
    
    # Apply low-pass filters to pitch_dot and wheel velocity
    # Filter formula: filtered = filtered * alpha + new_value * (1 - alpha)
    # Alpha = 0.975 gives smooth filtering
    self.pitch_dot_filtered = (self.pitch_dot_filtered * 0.975) + (self.pitch_dot * 0.025)
    self.velocity_angular_filtered = (self.velocity_angular_filtered * 0.975) + (self.wheel_velocity * 0.025)
    
    # Calculate velocity error (state x4)
    # Error = desired - actual
    velocity_linear_error = self.velocity_linear_set_point - self.velocity_angular_filtered * WHEEL_RADIUS
    
    # LQR Control Law: u = -K*x
    # State vector x = [pitch, pitch_dot, position, velocity]
    # We control: pitch (x1), pitch_dot (x2), and velocity (x4)
    # Position (x3) is not controlled, set to 0
    
    x1 = 0 - self.pitch  # Pitch error (desired pitch is 0 for upright)
    x2 = self.pitch_dot_filtered
    x3 = 0  # Position error (not controlled)
    x4 = velocity_linear_error
    
    # Calculate LQR control input
    # u = -K[0]*x1 - K[1]*x2 - K[2]*x3 - K[3]*x4
    # Note: The negative sign is already in the original code structure
    lqr_v = LQR_K[0] * x1 + LQR_K[1] * x2 + LQR_K[2] * x3 + LQR_K[3] * x4
    
    # Convert to wheel angular velocity (rad/s)
    wheel_vel = -lqr_v / WHEEL_RADIUS
    
    # Clamp to maximum motor velocity
    wheel_vel = clamp(wheel_vel, -MAX_MOTOR_VEL, MAX_MOTOR_VEL)
    
    # Apply control to motors
    # Left wheel: negative velocity + yaw control
    # Right wheel: positive velocity + yaw control
    # The yaw term allows the robot to turn
    self.sim.setJointTargetVelocity(self.left_joint_handle, wheel_vel )
    self.sim.setJointTargetVelocity(self.right_joint_handle, wheel_vel )
    
    #####################################


def sysCall_cleanup():
    ######## CLEANUP ######
    # Stop the motors when simulation ends
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    
    print("Self-Balancing Robot Controller Stopped")
    #######################


# Additional helper functions (optional - can be called from other scripts)

def set_velocity_linear_set_point(vel):
    """
    Sets the target velocity of the robot
    Example: set_velocity_linear_set_point(0.5) for 0.5 m/s forward
             set_velocity_linear_set_point(-0.3) for 0.3 m/s backward
    """
    self.velocity_linear_set_point = vel
    print(f"Target velocity set to: {vel} m/s")

def set_yaw(yaw_rate):
    """
    Sets the yaw rate for turning
    Example: set_yaw(2.0) to turn (positive or negative depending on convention)
             set_yaw(0) to stop turning
    """
    self.yaw = yaw_rate
    print(f"Yaw rate set to: {yaw_rate} rad/s")

def reset_controller():
    """
    Reset controller state variables
    """
    self.velocity_angular = 0.0
    self.velocity_linear_set_point = 0.0
    self.yaw = 0.0
    self.pitch_dot_filtered = 0.0
    self.velocity_angular_filtered = 0.0
    
    # Stop motors
    self.sim.setJointTargetVelocity(self.left_joint_handle, 0)
    self.sim.setJointTargetVelocity(self.right_joint_handle, 0)
    
    print("Controller reset")


"""
SETUP INSTRUCTIONS FOR YOUR SCENE:
===================================

1. JOINT CONFIGURATION:
   - Select 'left_joint' in scene hierarchy
   - In joint properties, set:
     * Joint mode: "Motor" (not "Force/Torque")
     * Control loop: Enable "Velocity control"
     * Motor enabled: Check this box
     * Target velocity: 0 (will be set by script)
     * Maximum torque: Set appropriate value (e.g., 10 N·m)
   
   - Repeat same settings for 'right_joint'

2. BODY CONFIGURATION:
   - 'body' should be:
     * Dynamic and respondable
     * Have appropriate mass and inertia
     * Connected to wheels via the joints

3. VERIFY WHEEL RADIUS:
   - Your wheels appear to be N20_Wheel_D43_mm_v2_1
   - Diameter = 43mm, so radius = 21.5mm = 0.0215m
   - UPDATE THIS LINE if needed: WHEEL_RADIUS = 0.0215
   - Current value is 0.034m - PLEASE VERIFY!

4. TEST WHEEL DIRECTION:
   - If robot moves backward when it should go forward, try:
     self.wheel_velocity = (vel_left + vel_right) / 2.0
   - Or swap the signs in setJointTargetVelocity

5. ATTACH THIS SCRIPT:
   - Right-click on any object (e.g., 'body' or 'Script')
   - Add -> Associated child script -> Non-threaded
   - Paste this entire code
   - Start simulation

USAGE:
======
- Robot will automatically balance when simulation starts
- To move forward: set_velocity_linear_set_point(0.5)
- To turn: set_yaw(2.0)
- To stop: set_velocity_linear_set_point(0) and set_yaw(0)

TROUBLESHOOTING:
================
1. Robot falls immediately:
   - Check WHEEL_RADIUS matches your actual wheel radius
   - Verify joint modes are set to "Motor" with velocity control
   - Check that body has reasonable mass/inertia

2. Robot oscillates:
   - Filter constants may need adjustment (try 0.95/0.05 instead of 0.975/0.025)
   - LQR gains may need retuning for your specific robot parameters

3. Robot moves in wrong direction:
   - Flip signs in wheel_velocity calculation or setJointTargetVelocity

4. Motors don't respond:
   - Verify max torque is set high enough in joint properties
   - Check that motor is enabled in joint settings
"""