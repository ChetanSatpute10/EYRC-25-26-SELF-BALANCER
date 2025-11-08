# KEEP THE COMMENT BLOCKS EXACTLY AS PROVIDED (do not change)
def sysCall_init():
    sim = require('sim')

    # do some initialization here
    # This function will be executed once when the simulation starts

    ######## ADD YOUR CODE HERE #######
    # Hint: Initialize the scene objects which you will require
    #       Initialize algorithm related variables here
    ##################################

    # --- initialization of global variables
    global SIM, robot_body, motor_left, motor_right
    global lqr_gain_matrix, wheel_radius, max_motor_velocity
    global linear_velocity_target, yaw_target
    global filtered_pitch_rate, filtered_wheel_velocity, previous_pitch
    global yaw_magnitude, linear_velocity_magnitude
    global accumulated_pitch_error, integral_gain, integral_clamp_limit
    global ARM_JOINT, PRISMATIC_JOINT
    global arm_handle, gripper_handle
    global ARM_SPEED, GRIPPER_SPEED

    SIM = sim
    robot_body = sim.getObject("/body")
    motor_left = sim.getObject("/left_joint")
    motor_right = sim.getObject("/right_joint")
    ARM_JOINT = sim.getObject("/arm_joint")
    PRISMATIC_JOINT = sim.getObject("/prismatic_joint")

    # controller parameters added (same LQR_K you provided)
    lqr_gain_matrix = [-2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827]
    wheel_radius = 0.0215
    max_motor_velocity = 225.0  # rad/s

    # Integral term parameters
    integral_gain = 0.0
    accumulated_pitch_error = 0.0
    integral_clamp_limit = 0.01  # limit for Anti-windup

    # State filters
    linear_velocity_target = 0.0
    yaw_target = 0.0
    filtered_pitch_rate = 0.0
    filtered_wheel_velocity = 0.0
    previous_pitch = 0.0

    # keyboard signal magnitudes
    yaw_magnitude = 1.5
    linear_velocity_magnitude = 0.2

    ARM_SPEED = 0.8
    GRIPPER_SPEED = 0.5

    # ensure motors start at zero velocity
    sim.setJointTargetVelocity(motor_left, 0)
    sim.setJointTargetVelocity(motor_right, 0)
    sim.setJointTargetVelocity(PRISMATIC_JOINT, 0)
    sim.setJointTargetVelocity(ARM_JOINT, 0)
    
    print("Self-Balancing Bot Initialized! Use keyboard to control:")
    print("Arrow Keys: Forward/Backward/Left/Right")
    print("W/S: Forward/Backward, Q/E: Left/Right")
    print("I/K: Arm Up/Down, O/L: Gripper Open/Close")


# ---------------- Actuation ----------------
def sysCall_actuation():
    global SIM, motor_left, motor_right
    global linear_velocity_target, yaw_target
    global yaw_magnitude, linear_velocity_magnitude, max_motor_velocity

    # --- Keyboard Input Detection ---
    message, data, data2 = SIM.getSimulatorMessage()
    if (message == SIM.message_keypress):
        key = data[0]

        # Arrow keys (up, down, left, right)
        if key == 2007:  # up arrow
            linear_velocity_target = linear_velocity_magnitude
        elif key == 2008:  # down arrow
            linear_velocity_target = -linear_velocity_magnitude
        elif key == 2009:  # left arrow
            yaw_target = yaw_magnitude
        elif key == 2010:  # right arrow
            yaw_target = -yaw_magnitude

        # Manual control keys (optional)
        elif key == 119:  # w
            linear_velocity_target = linear_velocity_magnitude
        elif key == 115:  # s
            linear_velocity_target = -linear_velocity_magnitude
        elif key == 113:  # q
            yaw_target = yaw_magnitude
        elif key == 101:  # e
            yaw_target = -yaw_magnitude
        else:
            # Stop if no movement key pressed
            linear_velocity_target = 0.0
            yaw_target = 0.0

    # Controller output (LQR + integral)
    control_velocity = compute_lqr_control_with_integral()

    # Apply yaw differential and clamp
    left_motor_velocity = clamp(control_velocity + yaw_target, -max_motor_velocity, max_motor_velocity)
    right_motor_velocity = clamp(control_velocity - yaw_target, -max_motor_velocity, max_motor_velocity)

    SIM.setJointTargetVelocity(motor_left, left_motor_velocity)
    SIM.setJointTargetVelocity(motor_right, right_motor_velocity)

    # Arm and gripper control
    global ARM_JOINT, PRISMATIC_JOINT, ARM_SPEED, GRIPPER_SPEED

    message, data, data2 = SIM.getSimulatorMessage()
    if (message == SIM.message_keypress):
        key = data[0]
        arm_vel = 0.0
        if key == 105:  # 'i' key to raise arm
            arm_vel = ARM_SPEED
        elif key == 107:  # 'k' key to lower arm
            arm_vel = -ARM_SPEED
        elif key == 32:
            arm_vel = 0.0
        SIM.setJointTargetVelocity(ARM_JOINT, arm_vel)
        gripper_vel = 0.0
        if key == 111:  # 'o' key to open gripper
            gripper_vel = GRIPPER_SPEED
        elif key == 108:  # 'l' key to close gripper
            gripper_vel = -GRIPPER_SPEED
        elif key == 32:
            gripper_vel = 0.0
        SIM.setJointTargetVelocity(PRISMATIC_JOINT, gripper_vel)


# ---------------- Sensing ----------------
def sysCall_sensing():
    global SIM, robot_body, motor_left, motor_right
    global filtered_pitch_rate, filtered_wheel_velocity, previous_pitch
    global accumulated_pitch_error, integral_clamp_limit

    # Get orientation in GLOBAL frame
    orientation = SIM.getObjectOrientation(robot_body, -1)
    roll_global = orientation[0] if orientation else 0.0
    pitch_global = orientation[1] if orientation else 0.0
    yaw_global = orientation[2] if orientation else 0.0

    # Convert global angular velocity to local frame
    _, angular_velocity_global = SIM.getObjectVelocity(robot_body)
    
    # Transform angular velocity from global to local frame
    # We need to rotate the angular velocity vector by the inverse of the robot's yaw
    import math
    cos_yaw = math.cos(-yaw_global)
    sin_yaw = math.sin(-yaw_global)
    
    # Rotate angular velocity to robot's local frame
    pitch_rate_local = (angular_velocity_global[0] * cos_yaw - 
                        angular_velocity_global[1] * sin_yaw)
    
    # Calculate local pitch angle from global orientation
    # The pitch in the robot's local frame (around its X-axis)
    local_pitch = math.atan2(
        math.sin(roll_global) * math.cos(yaw_global) + math.sin(pitch_global) * math.sin(yaw_global),
        math.cos(roll_global) * math.cos(pitch_global)
    )

    # wheel speeds
    left_wheel_speed = SIM.getJointVelocity(motor_left) or 0.0
    right_wheel_speed = SIM.getJointVelocity(motor_right) or 0.0
    avg_wheel_speed = (left_wheel_speed + right_wheel_speed) / 2.0

    # Update filters
    filtered_pitch_rate = (filtered_pitch_rate * 0.95) + (pitch_rate_local * 0.05)
    filtered_wheel_velocity = (filtered_wheel_velocity * 0.95) + (avg_wheel_speed * 0.05)

    # Update integral pitch error
    pitch_error = -local_pitch
    accumulated_pitch_error += pitch_error
    accumulated_pitch_error = clamp(accumulated_pitch_error, -integral_clamp_limit, integral_clamp_limit)

    previous_pitch = local_pitch


# ---------------- Cleanup ----------------
def sysCall_cleanup():
    global SIM, motor_left, motor_right
    try:
        SIM.setJointTargetVelocity(motor_left, 0)
        SIM.setJointTargetVelocity(motor_right, 0)
        print("Cleanup completed - motors stopped")
    except:
        pass


# ---------------- Helpers ----------------
def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def compute_lqr_control_with_integral():
    global lqr_gain_matrix, wheel_radius, linear_velocity_target
    global filtered_pitch_rate, filtered_wheel_velocity, previous_pitch
    global integral_gain, accumulated_pitch_error

    pitch = previous_pitch
    pitch_rate = filtered_pitch_rate
    velocity_error = linear_velocity_target - (filtered_wheel_velocity * wheel_radius)

    # LQR control
    lqr_output = (lqr_gain_matrix[0] * (-pitch) +
                  lqr_gain_matrix[1] * pitch_rate +
                  lqr_gain_matrix[2] * 0 +
                  lqr_gain_matrix[3] * velocity_error)

    # Integral correction
    integral_term = integral_gain * accumulated_pitch_error
    total_output = lqr_output + integral_term

    return -total_output / wheel_radius