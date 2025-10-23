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
    chassis_handle = sim.getObjectHandle("body")
    left_wheel_joint = sim.getObjectHandle("left_joint")
    right_wheel_joint = sim.getObjectHandle("right_joint")
    manipulator_joint = sim.getObjectHandle("arm_joint")
    gripper_actuator = sim.getObjectHandle("Prismatic_joint")

    # State-space feedback controller gains
    state_feedback_gains = [-2.8404703083006493, -0.04011592828490343, 
                            -3.3368304906550407e-17, 2.2360679774997827]
    wheel_rad = 0.0215
    velocity_limit = 225.0  

    ki_coefficient = 0.0
    pitch_error_sum = 0.0
    windup_threshold = 0.01

    desired_linear_vel = 0.0
    desired_angular_vel = 0.0
    smoothed_angular_rate = 0.0
    smoothed_wheel_angular_vel = 0.0
    last_tilt_angle = 0.0

    rotation_command_scale = 1.5
    translation_command_scale = 0.2

    manipulator_velocity = 0.8
    gripper_velocity = 0.5
    for joint in [left_wheel_joint, right_wheel_joint, gripper_actuator, manipulator_joint]:
        sim.setJointTargetVelocity(joint, 0)
    
    print("Self-Balancing Robot System Online!")
    print("Navigation: Arrow Keys or W/S (forward/back), Q/E (rotate)")
    print("Manipulation: I/K (arm), O/L (gripper)")


def sysCall_actuation():
    global simulation_api, left_wheel_joint, right_wheel_joint
    global desired_linear_vel, desired_angular_vel
    global rotation_command_scale, translation_command_scale, velocity_limit

    msg_type, msg_data, _ = simulation_api.getSimulatorMessage()
    
    if msg_type == simulation_api.message_keypress:
        pressed_key = msg_data[0]
        desired_linear_vel, desired_angular_vel = process_navigation_input(pressed_key)

    base_wheel_command = calculate_stabilization_command()

    left_cmd = constrain_value(base_wheel_command + desired_angular_vel, 
                                -velocity_limit, velocity_limit)
    right_cmd = constrain_value(base_wheel_command - desired_angular_vel, 
                                 -velocity_limit, velocity_limit)

    simulation_api.setJointTargetVelocity(left_wheel_joint, left_cmd)
    simulation_api.setJointTargetVelocity(right_wheel_joint, right_cmd)

    handle_manipulator_commands()


def process_navigation_input(key_code):
    """Process keyboard input and return velocity commands"""
    global translation_command_scale, rotation_command_scale
    
    nav_mapping = {
        2007: (translation_command_scale, 0.0),    # up arrow
        2008: (-translation_command_scale, 0.0),   # down arrow
        2009: (0.0, rotation_command_scale),       # left arrow
        2010: (0.0, -rotation_command_scale),      # right arrow
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
        if key == 105:  # i
            arm_cmd = manipulator_velocity
        elif key == 107:  # k
            arm_cmd = -manipulator_velocity
        simulation_api.setJointTargetVelocity(manipulator_joint, arm_cmd)
        
        grip_cmd = 0.0
        if key == 111:  # o
            grip_cmd = gripper_velocity
        elif key == 108:  # l
            grip_cmd = -gripper_velocity
        simulation_api.setJointTargetVelocity(gripper_actuator, grip_cmd)


def sysCall_sensing():
    global simulation_api, chassis_handle, left_wheel_joint, right_wheel_joint
    global smoothed_angular_rate, smoothed_wheel_angular_vel, last_tilt_angle
    global pitch_error_sum, windup_threshold

    euler_angles = simulation_api.getObjectOrientation(chassis_handle, -1)
    roll_world = euler_angles[0] if euler_angles else 0.0
    pitch_world = euler_angles[1] if euler_angles else 0.0
    yaw_world = euler_angles[2] if euler_angles else 0.0

    _, omega_world = simulation_api.getObjectVelocity(chassis_handle)
    
    cos_heading = cos_approx(-yaw_world)
    sin_heading = sin_approx(-yaw_world)
    
    pitch_rate_body = (omega_world[0] * cos_heading - omega_world[1] * sin_heading)
    
    numerator = sin_approx(roll_world) * cos_approx(yaw_world) + sin_approx(pitch_world) * sin_approx(yaw_world)
    denominator = cos_approx(roll_world) * cos_approx(pitch_world)
    tilt_angle_body = atan2_approx(numerator, denominator)

    left_encoder = simulation_api.getJointVelocity(left_wheel_joint) or 0.0
    right_encoder = simulation_api.getJointVelocity(right_wheel_joint) or 0.0
    mean_wheel_rate = (left_encoder + right_encoder) * 0.5

    alpha = 0.05
    smoothed_angular_rate = smoothed_angular_rate * (1 - alpha) + pitch_rate_body * alpha
    smoothed_wheel_angular_vel = smoothed_wheel_angular_vel * (1 - alpha) + mean_wheel_rate * alpha

    current_pitch_error = -tilt_angle_body
    pitch_error_sum = constrain_value(pitch_error_sum + current_pitch_error, 
                                       -windup_threshold, windup_threshold)

    last_tilt_angle = tilt_angle_body


def sysCall_cleanup():
    global simulation_api, left_wheel_joint, right_wheel_joint
    try:
        simulation_api.setJointTargetVelocity(left_wheel_joint, 0)
        simulation_api.setJointTargetVelocity(right_wheel_joint, 0)
        print("System shutdown - actuators disabled")
    except:
        pass


def constrain_value(val, lower_bound, upper_bound):
    """Saturate value within specified bounds"""
    return max(lower_bound, min(upper_bound, val))


def sin_approx(angle):
    """Approximation of sine using Taylor series (for small angles)"""
  
    pi = 3.14159265359
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    
    
    x2 = angle * angle
    return angle * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0))


def cos_approx(angle):
    """Approximation of cosine using Taylor series"""
   
    pi = 3.14159265359
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    
  
    x2 = angle * angle
    return 1.0 - x2 / 2.0 * (1.0 - x2 / 12.0)


def atan2_approx(y, x):
    """Approximation of atan2 function"""
    pi = 3.14159265359
    
    if x == 0.0:
        if y > 0:
            return pi / 2.0
        elif y < 0:
            return -pi / 2.0
        else:
            return 0.0
    
    abs_y = abs(y)
    abs_x = abs(x)
    
    
    if abs_x > abs_y:
        ratio = y / x
        angle = ratio / (1.0 + 0.28 * ratio * ratio)
        if x < 0:
            if y >= 0:
                angle += pi
            else:
                angle -= pi
    else:
        ratio = x / y
        angle = pi / 2.0 - ratio / (1.0 + 0.28 * ratio * ratio)
        if y < 0:
            angle -= pi
    
    return angle


def calculate_stabilization_command():
   
    global state_feedback_gains, wheel_rad, desired_linear_vel
    global smoothed_angular_rate, smoothed_wheel_angular_vel, last_tilt_angle
    global ki_coefficient, pitch_error_sum

    current_tilt = last_tilt_angle
    tilt_rate = smoothed_angular_rate
    vel_tracking_error = desired_linear_vel - (smoothed_wheel_angular_vel * wheel_rad)

   
    feedback_term = (state_feedback_gains[0] * (-current_tilt) +
                     state_feedback_gains[1] * tilt_rate +
                     state_feedback_gains[2] * 0 +
                     state_feedback_gains[3] * vel_tracking_error)

   
    integral_compensation = ki_coefficient * pitch_error_sum
    control_output = feedback_term + integral_compensation

    return -control_output / wheel_rad