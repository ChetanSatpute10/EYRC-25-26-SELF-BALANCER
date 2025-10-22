import keyboard
import sys
import time
from coppeliasim_zmqremoteapi_client import *

# LQR_K = [ -2.4313702697352912, -0.024102528121525937, 1.805403236315983e-17, 2.2360679774997942]
LQR_K = [ -2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827]
WHEEL_RADIUS = 0.0215
MAX_MOTOR_VEL = 10.0 # rad/s

velocity_linear_set_point = 0.0
yaw = 0

velocity_angular = 0.0
last_pitch = 0.0
pitch_dot_filtered = 0.0
velocity_angular_filtered = 0.0

current_keys = {
    'up': False,
    'down': False,
    'left': False,
    'right': False
}
client = RemoteAPIClient('localhost', 23000)
sim = client.require('sim')
body = sim.getObjectHandle("body")
left_joint = sim.getObjectHandle("left_joint")
right_joint = sim.getObjectHandle("right_joint")

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def get_pitch() -> float:
    """Get the pitch angle (rotation around X-axis) in radians"""
    orientation = sim.getObjectOrientation(body, -1)
    return orientation[0]  # X-axis rotation (pitch)

def get_pitch_dot() -> float:
    """Get the angular velocity around X-axis (pitch rate) in rad/s"""
    linear_vel, angular_vel = sim.getObjectVelocity(body)
    return angular_vel[0]  # X-axis angular velocity

def get_wheel_velocity() -> float:
    """Get average wheel velocity in rad/s"""
    vel_left = sim.getJointVelocity(left_joint)
    vel_right = sim.getJointVelocity(right_joint)
    # Assuming one wheel is reversed
    return (vel_left + vel_right) / 2.0

def calculate_lqr_velocity() -> float:
    global pitch_dot_filtered, velocity_angular_filtered
    global velocity_linear_set_point, yaw

    pitch = get_pitch()
    pitch_dot = get_pitch_dot()
    # apply a filter to pitch dot, and velocity
    # without these filters the controller seems to lack necessary dampening
    # would like to know why!
    pitch_dot_filtered = (pitch_dot_filtered * .975) + (pitch_dot * .025)
    velocity_angular_filtered = (velocity_angular_filtered * .975) + (get_wheel_velocity() * .025)
    velocity_linear_error = velocity_linear_set_point - velocity_angular_filtered * WHEEL_RADIUS
    lqr_v = LQR_K[0] * (0 - pitch) + LQR_K[1] * pitch_dot_filtered + LQR_K[2] * 0 + LQR_K[3] * velocity_linear_error
    return -lqr_v / WHEEL_RADIUS

def reset_all_keys():
    for key in current_keys:
        current_keys[key] = False

def on_key_press(event):
    if event.name in ['up', 'w']:
        reset_all_keys()
        current_keys['up'] = True
        print("UP")
    elif event.name in ['down', 's']:
        reset_all_keys()
        current_keys['down'] = True
        print("DOWN")
    elif event.name in ['left', 'a']:
        reset_all_keys()
        current_keys['left'] = True
        print("LEFT")
    elif event.name in ['right', 'd']:
        reset_all_keys()
        current_keys['right'] = True
        print("RIGHT")
    elif event.name == 'esc':
        print("Exiting...")
        sys.exit()

def on_key_release(event):
    if event.name in ['up', 'w']:
        current_keys['up'] = False
    elif event.name in ['down', 's']:
        current_keys['down'] = False
    elif event.name in ['left', 'a']:
        current_keys['left'] = False
    elif event.name in ['right', 'd']:
        current_keys['right'] = False

def init():
    print('[INFO] Connecting to CoppeliaSim...')
    print('[INFO] Starting simulation...')
    
    sim.setJointTargetVelocity(left_joint, 0)
    sim.setJointTargetVelocity(right_joint, 0)
    sim.startSimulation()

def move(L_motor, R_motor):
    sim.setJointTargetVelocity(left_joint, L_motor)
    sim.setJointTargetVelocity(right_joint, R_motor)

def loop():
    global yaw, velocity_linear_set_point
    # orientation = sim.getObjectOrientation(body)
    vel = calculate_lqr_velocity()
    vel = clamp(vel + yaw, -MAX_MOTOR_VEL, MAX_MOTOR_VEL)
    sim.setJointTargetVelocity(left_joint, vel + yaw)
    sim.setJointTargetVelocity(right_joint, vel - yaw)

    # print(orientation[0])
    
    # time.sleep(0.1)

    if current_keys['up']:
        # move(10, 10)
        velocity_linear_set_point = 0.01
        
    elif current_keys['down']:
        # move(-10, -10)
        velocity_linear_set_point = 0.0
        yaw = 0.0

    elif current_keys['left']:
        # move(-5, 5)
        yaw = 0.1

    # elif current_keys['right']:
    #     move(5, -5)

    # else:
    #     move(0, 0)

def main():
    init()
    try:
        keyboard.on_press(on_key_press)
        keyboard.on_release(on_key_release)
        
        while True:
            loop()
            
    except KeyboardInterrupt:
        print('[INFO] Stopping simulation...')
        sim.stopSimulation()
        print("Exiting...")
        sys.exit()

if __name__ == "__main__":
    main()