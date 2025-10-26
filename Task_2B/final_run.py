# KEEP THE COMMENT BLOCKS EXACTLY AS PROVIDED (do not change)
def sysCall_init():
    sim = require('sim')

    # do some initialization here
    # This function will be executed once when the simulation starts

    ######## ADD YOUR CODE HERE #######
    # Hint: Initialize the scene objects which you will require
    #       Initialize algorithm related variables here
    ##################################

    global simulation_api, chassis_handle, left_wheel_joint, right_wheel_joint
    global state_feedback_gains, wheel_rad, velocity_limit
    global desired_linear_vel, desired_angular_vel
    global smoothed_angular_rate, smoothed_wheel_angular_vel, last_tilt_angle
    global rotation_command_scale, translation_command_scale
    global pitch_error_sum, ki_coefficient, windup_threshold
    global manipulator_joint, gripper_actuator
    global manipulator_velocity, gripper_velocity

    simulation_api = sim
    chassis_handle = sim.getObject("/body")
    left_wheel_joint = sim.getObject("/left_joint")
    right_wheel_joint = sim.getObject("/right_joint")
    manipulator_joint = sim.getObject("/arm_joint")
    gripper_actuator = sim.getObject("/Prismatic_joint")

    # State-space feedback controller gains
    state_feedback_gains = [-2.8404703083006493, -0.04011592828490343, 
                            -3.3368304906550407e-17, 2.2360679774997827]
    wheel_rad = 0.0215
    velocity_limit = 180.0  # Maximum angular velocity in rad/s

    # Anti-windup integral control parameters
    ki_coefficient = 0.0
    pitch_error_sum = 0.0
    windup_threshold = 0.01

    # Smoothed state variables
    desired_linear_vel = 0.0
    desired_angular_vel = 0.0
    smoothed_angular_rate = 0.0
    smoothed_wheel_angular_vel = 0.0
    last_tilt_angle = 0.0

    # Command scaling factors
    rotation_command_scale = 1.5
    translation_command_scale = 0.12

    manipulator_velocity = 1.0
    gripper_velocity = 0.5

    
    for joint in [left_wheel_joint, right_wheel_joint, gripper_actuator, manipulator_joint]:
        sim.setJointTargetVelocity(joint, 0)
    
    print(f"w,s for forward/backward while q,e for turning left/right and h.n for raising/lowering the manipulator")


def sysCall_actuation():
    global simulation_api, left_wheel_joint, right_wheel_joint
    global desired_linear_vel, desired_angular_vel
    global rotation_command_scale, translation_command_scale, velocity_limit

    # Process keyboard commands
    msg_type, msg_data, _ = simulation_api.getSimulatorMessage()
    
    if msg_type == simulation_api.message_keypress:
        pressed_key = msg_data[0]
        desired_linear_vel, desired_angular_vel = process_navigation_input(pressed_key)

    # Compute stabilization control output
    base_wheel_command = calculate_stabilization_command()

    # Apply differential drive and saturation
    left_cmd = constrain_value(base_wheel_command + desired_angular_vel, 
                                -velocity_limit, velocity_limit)
    right_cmd = constrain_value(base_wheel_command - desired_angular_vel, 
                                 -velocity_limit, velocity_limit)

    simulation_api.setJointTargetVelocity(left_wheel_joint, left_cmd)
    simulation_api.setJointTargetVelocity(right_wheel_joint, right_cmd)

    # Handle manipulator controls
    handle_manipulator_commands()

def constrain_value(val, lower_bound, upper_bound):
    """Saturate value within specified bounds"""
    return max(lower_bound, min(upper_bound, val))


def calculate_stabilization_command():
    """Compute control action using state feedback with integral compensation"""
    global state_feedback_gains, wheel_rad, desired_linear_vel
    global smoothed_angular_rate, smoothed_wheel_angular_vel, last_tilt_angle
    global ki_coefficient, pitch_error_sum

    current_tilt = last_tilt_angle
    tilt_rate = smoothed_angular_rate
    vel_tracking_error = desired_linear_vel - (smoothed_wheel_angular_vel * wheel_rad)

    # State feedback control law: u = -K*x
    feedback_term = (state_feedback_gains[0] * (-current_tilt) +
                     state_feedback_gains[1] * tilt_rate +
                     state_feedback_gains[2] * 0 +
                     state_feedback_gains[3] * vel_tracking_error)

    # Add integral compensation
    integral_compensation = ki_coefficient * pitch_error_sum
    control_output = feedback_term + integral_compensation

    return -control_output / wheel_rad
def process_navigation_input(key_code):
    """Process keyboard input and return velocity commands"""
    global translation_command_scale, rotation_command_scale
    
    nav_mapping = {
        119: (translation_command_scale, 0.0),     # w
        115: (-translation_command_scale, 0.0),    # s
        113: (0.0, rotation_command_scale),        # q
        101: (0.0, -rotation_command_scale),       # e
    }
    
    return nav_mapping.get(key_code, (0.0, 0.0))


def handle_manipulator_commands():
    """Control arm and gripper based on keyboard input"""
    global simulation_api, manipulator_joint, gripper_actuator
    global manipulator_velocity, gripper_velocity
    
    msg_type, msg_data, _ = simulation_api.getSimulatorMessage()
    
    if msg_type == simulation_api.message_keypress:
        key = msg_data[0]
        
       
        arm_cmd = 0.0
        if key == 104:  #h key for raising the arm
            arm_cmd = manipulator_velocity
        elif key == 110: #n key for lowering the arm
            arm_cmd = -manipulator_velocity
        simulation_api.setJointTargetVelocity(manipulator_joint, arm_cmd)
def sysCall_sensing():
    global simulation_api, chassis_handle, left_wheel_joint, right_wheel_joint
    global smoothed_angular_rate, smoothed_wheel_angular_vel, last_tilt_angle
    global pitch_error_sum, windup_threshold
    import math

    # Retrieve chassis orientation in world frame
    euler_angles = simulation_api.getObjectOrientation(chassis_handle, -1)
    roll_world = euler_angles[0] if euler_angles else 0.0
    pitch_world = euler_angles[1] if euler_angles else 0.0
    yaw_world = euler_angles[2] if euler_angles else 0.0

    # Get angular velocity in world frame and transform to body frame
    _, omega_world = simulation_api.getObjectVelocity(chassis_handle)
    
    # Proper rotation matrix transformation (inverted yaw rotation)
    cos_heading = math.cos(-yaw_world)
    sin_heading = math.sin(-yaw_world)
    
    # Transform angular velocity to body frame (rotation about local X-axis)
    pitch_rate_body = (omega_world[0] * cos_heading - omega_world[1] * sin_heading)
    
    # Compute local pitch angle from global orientation
    # This properly handles 360-degree rotations
    tilt_angle_body = math.atan2(
        math.sin(roll_world) * math.cos(yaw_world) + math.sin(pitch_world) * math.sin(yaw_world),
        math.cos(roll_world) * math.cos(pitch_world)
    )

    # Read wheel encoders
    left_encoder = simulation_api.getJointVelocity(left_wheel_joint) or 0.0
    right_encoder = simulation_api.getJointVelocity(right_wheel_joint) or 0.0
    mean_wheel_rate = (left_encoder + right_encoder) * 0.5

    # Apply exponential smoothing filters
    alpha = 0.05
    smoothed_angular_rate = smoothed_angular_rate * (1.0 - alpha) + pitch_rate_body * alpha
    smoothed_wheel_angular_vel = smoothed_wheel_angular_vel * (1.0 - alpha) + mean_wheel_rate * alpha

    # Integral accumulation with anti-windup
    current_pitch_error = -tilt_angle_body
    pitch_error_sum = constrain_value(pitch_error_sum + current_pitch_error, 
                                       -windup_threshold, windup_threshold)

    last_tilt_angle = tilt_angle_body


def sysCall_cleanup():
    global simulation_api, left_wheel_joint, right_wheel_joint
    try:
        simulation_api.setJointTargetVelocity(left_wheel_joint, 0)
        simulation_api.setJointTargetVelocity(right_wheel_joint, 0)
        print("The run is done")
    except:
        pass
