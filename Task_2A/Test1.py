# KEEP THE COMMENT BLOCKS EXACTLY AS PROVIDED (do not change)
def sysCall_init():
    sim = require('sim')
    simUI = require('simUI')

    # do some initialization here
    # This function will be executed once when the simulation starts
    
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000
    
    ######## ADD YOUR CODE HERE #######
    # Hint: Initialize the scene objects which you will require
    #       Initialize algorithm related variables here
    
    ##################################

    # --- initialization of globals (safe and robust in CoppeliaSim child scripts)
    global SIM, SIMUI, BODY, LEFT_JOINT, RIGHT_JOINT
    global LQR_K, WHEEL_RADIUS, MAX_MOTOR_VEL
    global velocity_linear_set_point, yaw_cmd
    global pitch_dot_filtered, velocity_angular_filtered, last_pitch
    global YAW_MAG, VEL_SETPOINT_MAG, yaw_comp_factor
    global integral_error, Ki, integral_limit
    global ui_handle, ui_forward, ui_backward, ui_left, ui_right, ui_stop
    global ARM_JOINT, PRISMATIC_JOINT
    global arm_handle, gripper_handle
    global ui_raise, ui_lower, ui_grab, ui_drop, ui_stop_gripper, ui_stop_arm
    global ARM_SPEED, GRIPPER_SPEED

    SIM = sim
    SIMUI = simUI
    BODY = sim.getObjectHandle("body")
    LEFT_JOINT = sim.getObjectHandle("left_joint")
    RIGHT_JOINT = sim.getObjectHandle("right_joint")
    
    # Arm manipulator joints
    ARM_JOINT = sim.getObjectHandle("arm_joint")
    PRISMATIC_JOINT = sim.getObjectHandle("Prismatic_joint")

    # controller parameters (same LQR_K you provided)
    LQR_K = [ -2.8404703083006493, -0.04011592828490343, -3.3368304906550407e-17, 2.2360679774997827 ]
    WHEEL_RADIUS = 0.0215
    MAX_MOTOR_VEL = 210.0 # rad/s

    # Integral term parameters (tune these to reduce wobbling)
    Ki = 0.0  # Integral gain - increase to reduce steady-state error, decrease if oscillations increase
    integral_error = 0.0
    integral_limit = 0.02  # Anti-windup limit

    # state / filters
    velocity_linear_set_point = 0.0
    yaw_cmd = 0.0
    pitch_dot_filtered = 0.0
    velocity_angular_filtered = 0.0
    last_pitch = 0.0

    # keyboard -> signal magnitudes (tweak as needed)
    YAW_MAG = 0.75
    VEL_SETPOINT_MAG = 0.01

    # yaw compensation heuristic factor (tweak if needed)
    yaw_comp_factor = 0.5

    # Arm control parameters - ADJUST THESE TO CHANGE SPEED
    ARM_SPEED = 1.0  # rad/s - Speed for raising/lowering arm
    GRIPPER_SPEED = 0.5  # m/s - Speed for opening/closing gripper
    
    # Arm control button states
    ui_raise = False
    ui_lower = False
    ui_grab = False
    ui_drop = False
    ui_stop_gripper = False
    ui_stop_arm = False

    # Control button states
    ui_forward = False
    ui_backward = False
    ui_left = False
    ui_right = False
    ui_stop = False

    # ensure motors start at zero
    sim.setJointTargetVelocity(LEFT_JOINT, 0)
    sim.setJointTargetVelocity(RIGHT_JOINT, 0)
    sim.setJointTargetVelocity(ARM_JOINT, 0)
    sim.setJointTargetVelocity(PRISMATIC_JOINT, 0)
    
    # Create UI control panels
    ui_handle = create_control_ui()
    arm_handle = create_arm_control_ui()
    gripper_handle = create_gripper_control_ui()
    
    print("Self-Balancing Bot Initialized!")
    print("Use the Control Panel to move the robot")
    print("Use the Arm Control Panel to control the manipulator")
    print("Integral Control Active (Ki = {})".format(Ki))
    print("Arm Speed: {} rad/s | Gripper Speed: {} m/s".format(ARM_SPEED, GRIPPER_SPEED))

def create_control_ui():
    """Create a simple UI panel for robot control"""
    global SIMUI
    
    xml = '''
    <ui title="Robot Control" closeable="true" resizable="false" activate="true">
        <group layout="grid" flat="true">
            <label text="" style="* {min-width: 100px; min-height: 50px;}" grid-row="0" grid-column="0"/>
            <button text="⬆ Forward" on-click="onForwardPressed" on-release="onForwardReleased" 
                    style="* {min-width: 100px; min-height: 50px; font-size: 16px;}" 
                    grid-row="0" grid-column="1"/>
            <label text="" style="* {min-width: 100px; min-height: 50px;}" grid-row="0" grid-column="2"/>
            
            <button text="⬅ Left" on-click="onLeftPressed" on-release="onLeftReleased" 
                    style="* {min-width: 100px; min-height: 50px; font-size: 16px;}" 
                    grid-row="1" grid-column="0"/>
            <button text="STOP" on-click="onStopPressed" 
                    style="* {min-width: 100px; min-height: 50px; font-size: 16px; background-color: #ff6b6b;}" 
                    grid-row="1" grid-column="1"/>
            <button text="Right ➡" on-click="onRightPressed" on-release="onRightReleased" 
                    style="* {min-width: 100px; min-height: 50px; font-size: 16px;}" 
                    grid-row="1" grid-column="2"/>
            
            <label text="" style="* {min-width: 100px; min-height: 50px;}" grid-row="2" grid-column="0"/>
            <button text="⬇ Backward" on-click="onBackwardPressed" on-release="onBackwardReleased" 
                    style="* {min-width: 100px; min-height: 50px; font-size: 16px;}" 
                    grid-row="2" grid-column="1"/>
            <label text="" style="* {min-width: 100px; min-height: 50px;}" grid-row="2" grid-column="2"/>
        </group>
    </ui>
    '''
    
    try:
        return SIMUI.create(xml)
    except:
        print("Warning: Could not create UI panel. Using signal-based control.")
        return None

def create_arm_control_ui():
    """Create UI panel for arm control"""
    global SIMUI
    
    xml = '''
    <ui title="Arm Control" closeable="true" resizable="false" activate="true">
        <group layout="vbox" flat="true">
            <button text="↑ RAISE ARM" on-click="onRaisePressed" on-release="onRaiseReleased" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #4CAF50;}" />
            <button text="↓ LOWER ARM" on-click="onLowerPressed" on-release="onLowerReleased" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #2196F3;}" />
            <button text="⏸ STOP ARM" on-click="onStopArmPressed" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #f44336;}" />
        </group>
    </ui>
    '''
    
    try:
        return SIMUI.create(xml)
    except:
        print("Warning: Could not create Arm Control UI panel.")
        return None

def create_gripper_control_ui():
    """Create UI panel for gripper control"""
    global SIMUI
    
    xml = '''
    <ui title="Gripper Control" closeable="true" resizable="false" activate="true">
        <group layout="vbox" flat="true">
            <button text="✋ GRAB (Close)" on-click="onGrabPressed" on-release="onGrabReleased" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #FF9800;}" />
            <button text="✋ DROP (Open)" on-click="onDropPressed" on-release="onDropReleased" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #9C27B0;}" />
            <button text="⏸ STOP GRIPPER" on-click="onStopGripperPressed" 
                    style="* {min-width: 150px; min-height: 50px; font-size: 16px; background-color: #f44336;}" />
        </group>
    </ui>
    '''
    
    try:
        return SIMUI.create(xml)
    except:
        print("Warning: Could not create Gripper Control UI panel.")
        return None

def onForwardPressed(ui, id):
    global ui_forward
    ui_forward = True

def onForwardReleased(ui, id):
    global ui_forward
    ui_forward = False

def onBackwardPressed(ui, id):
    global ui_backward
    ui_backward = True

def onBackwardReleased(ui, id):
    global ui_backward
    ui_backward = False

def onLeftPressed(ui, id):
    global ui_left
    ui_left = True

def onLeftReleased(ui, id):
    global ui_left
    ui_left = False

def onRightPressed(ui, id):
    global ui_right
    ui_right = True

def onRightReleased(ui, id):
    global ui_right
    ui_right = False

def onStopPressed(ui, id):
    global ui_forward, ui_backward, ui_left, ui_right, velocity_linear_set_point, yaw_cmd
    ui_forward = False
    ui_backward = False
    ui_left = False
    ui_right = False
    velocity_linear_set_point = 0.0
    yaw_cmd = 0.0

# Arm control button handlers
def onRaisePressed(ui, id):
    global ui_raise
    ui_raise = True

def onRaiseReleased(ui, id):
    global ui_raise
    ui_raise = False

def onLowerPressed(ui, id):
    global ui_lower
    ui_lower = True

def onLowerReleased(ui, id):
    global ui_lower
    ui_lower = False

# Gripper control button handlers
def onGrabPressed(ui, id):
    global ui_grab
    ui_grab = True

def onGrabReleased(ui, id):
    global ui_grab
    ui_grab = False

def onDropPressed(ui, id):
    global ui_drop
    ui_drop = True

def onDropReleased(ui, id):
    global ui_drop
    ui_drop = False

def onStopGripperPressed(ui, id):
    global ui_grab, ui_drop
    ui_grab = False
    ui_drop = False

def onStopArmPressed(ui, id):
    global ui_raise, ui_lower
    ui_raise = False
    ui_lower = False

def sysCall_actuation():
    # put your actuation code here
    # This function will be executed at each simulation time step

    ####### ADD YOUR CODE HERE ######
    # Hint: Use the error feedback and apply control algorithm here
    #       Provide the resulting actuation as input to the actuator joint
    
    # Example psuedo code:
    #   x1 = error_state_1; # Error in states w.r.t desired setpoint
    #   x2 = error_state_2;
    #   x3 = error_state_3;
    #   x4 = error_state_4;
    #   k = [gain_1 , gain_1, gain_3, gain_4];      # These gains will be generated by control algorithm. For ex: LQR, PID, etc.
    #   U = -k[1]*x1 +k[2]*x2 -k[3]*x3 +k[4]*x4;    # +/- Sign convention may differ according to implementation
    #   Set_joint_actuation(U);                     # Provide this calculated input to system's actuator

    #################################
    global SIM, LEFT_JOINT, RIGHT_JOINT
    global velocity_linear_set_point, yaw_cmd
    global YAW_MAG, VEL_SETPOINT_MAG, MAX_MOTOR_VEL
    global ui_forward, ui_backward, ui_left, ui_right

    # # Primary: Use UI button states
    # up = 1 if ui_forward else 0
    # down = 1 if ui_backward else 0
    # left = 1 if ui_left else 0
    # right = 1 if ui_right else 0

    # # Fallback: Try reading from signals (if someone sets them externally)
    # if not (up or down or left or right):
    #     up_val = SIM.getIntegerSignal('key_up')
    #     down_val = SIM.getIntegerSignal('key_down')
    #     left_val = SIM.getIntegerSignal('key_left')
    #     right_val = SIM.getIntegerSignal('key_right')

    #     up = 1 if up_val is not None and up_val != 0 else 0
    #     down = 1 if down_val is not None and down_val != 0 else 0
    #     left = 1 if left_val is not None and left_val != 0 else 0
    #     right = 1 if right_val is not None and right_val != 0 else 0

    # setpoint & yaw based on keyboard input
    if up == 1:
        velocity_linear_set_point = VEL_SETPOINT_MAG
    elif down == 1:
        velocity_linear_set_point = -VEL_SETPOINT_MAG
    else:
        velocity_linear_set_point = 0.0

    if left == 1:
        yaw_cmd = YAW_MAG
    elif right == 1:
        yaw_cmd = -YAW_MAG
    else:
        yaw_cmd = 0.0

    # compute controller output with integral term (wheel angular velocity)
    vel = calculate_lqr_velocity_with_integral()

    # apply yaw differential and clamp
    vel_l = clamp(vel + yaw_cmd, -MAX_MOTOR_VEL, MAX_MOTOR_VEL)
    vel_r = clamp(vel - yaw_cmd, -MAX_MOTOR_VEL, MAX_MOTOR_VEL)

    SIM.setJointTargetVelocity(LEFT_JOINT, vel_l)
    SIM.setJointTargetVelocity(RIGHT_JOINT, vel_r)

    # ===== ARM MANIPULATOR CONTROL =====
    global ARM_JOINT, PRISMATIC_JOINT, ARM_SPEED, GRIPPER_SPEED
    global ui_raise, ui_lower, ui_grab, ui_drop
    
    # Arm raise/lower control
    arm_vel = 0.0
    if ui_raise:
        arm_vel = ARM_SPEED
    elif ui_lower:
        arm_vel = -ARM_SPEED
    
    SIM.setJointTargetVelocity(ARM_JOINT, arm_vel)
    
    # Gripper open/close control
    gripper_vel = 0.0
    if ui_grab:
        gripper_vel = -GRIPPER_SPEED  # Negative to close gripper
    elif ui_drop:
        gripper_vel = GRIPPER_SPEED  # Positive to open gripper
    
    SIM.setJointTargetVelocity(PRISMATIC_JOINT, gripper_vel)

def sysCall_sensing():
    # put your sensing code here
    # This function will be executed at each simulation time step
    
    ####### ADD YOUR CODE HERE ######
    # Hint: Take feedback here & do the error calculation
    
    #################################
    global SIM, BODY, LEFT_JOINT, RIGHT_JOINT
    global pitch_dot_filtered, velocity_angular_filtered, last_pitch
    global yaw_comp_factor, integral_error, integral_limit
    global ui_left, ui_right

    # orientation: [rotX, rotY, rotZ] (rotX is pitch for your robot)
    ori = SIM.getObjectOrientation(BODY, -1)
    pitch_raw = ori[0] if (ori is not None and len(ori) > 0) else 0.0

    # angular velocity: (linearVel, angularVel)
    lin_vel, ang_vel = SIM.getObjectVelocity(BODY)
    pitch_dot = ang_vel[0] if (ang_vel is not None and len(ang_vel) > 0) else 0.0

    # wheel velocities (rad/s)
    vel_left = SIM.getJointVelocity(LEFT_JOINT)
    vel_right = SIM.getJointVelocity(RIGHT_JOINT)
    if vel_left is None: vel_left = 0.0
    if vel_right is None: vel_right = 0.0
    wheel_avg = (vel_left + vel_right) / 2.0
    wheel_diff = vel_left - vel_right

    # UI-based yaw estimate
    yaw_key = (1 if ui_left else 0) - (1 if ui_right else 0)
    
    # Fallback to signals if UI not active
    if yaw_key == 0:
        left_sig = SIM.getIntegerSignal('key_left')
        right_sig = SIM.getIntegerSignal('key_right')
        yaw_key = (1 if (left_sig is not None and left_sig != 0) else 0) - (1 if (right_sig is not None and right_sig != 0) else 0)

    # apply small heuristic compensation to pitch to reduce yaw contamination
    pitch_corrected = pitch_raw - (yaw_comp_factor * (wheel_diff * 0.01 + yaw_key * 0.02))

    # update filters (same smoothing coefficients as original)
    pitch_dot_filtered = (pitch_dot_filtered * .975) + (pitch_dot * .025)
    velocity_angular_filtered = (velocity_angular_filtered * .975) + (wheel_avg * .025)

    # Update integral error (pitch error accumulation)
    pitch_error = 0 - pitch_corrected
    integral_error += pitch_error
    
    # Anti-windup: clamp the integral term
    integral_error = clamp(integral_error, -integral_limit, integral_limit)

    last_pitch = pitch_corrected

def sysCall_cleanup():
    # do some clean-up here
    # This function will be executed when the simulation ends
    
    ####### ADD YOUR CODE HERE ######
    # Any cleanup (if required) to take the scene back to it's original state after simulation
    # It helps in case simulation fails in an unwanted state.
    #################################
    global SIM, LEFT_JOINT, RIGHT_JOINT, ARM_JOINT, PRISMATIC_JOINT
    global ui_handle, arm_handle, gripper_handle, SIMUI
    try:
        SIM.setJointTargetVelocity(LEFT_JOINT, 0)
        SIM.setJointTargetVelocity(RIGHT_JOINT, 0)
        SIM.setJointTargetVelocity(ARM_JOINT, 0)
        SIM.setJointTargetVelocity(PRISMATIC_JOINT, 0)
        
        if ui_handle is not None:
            SIMUI.destroy(ui_handle)
        if arm_handle is not None:
            SIMUI.destroy(arm_handle)
        if gripper_handle is not None:
            SIMUI.destroy(gripper_handle)
            
        print("Cleanup completed - all motors stopped, UI panels closed")
    except:
        pass

# -----------------------
# helper functions and controller calculation (module-level)
# -----------------------
def clamp(n, minn, maxn):
    if n < minn: return minn
    if n > maxn: return maxn
    return n

def calculate_lqr_velocity_with_integral():
    # uses the filtered & corrected globals set in sensing
    global LQR_K, WHEEL_RADIUS, velocity_linear_set_point
    global pitch_dot_filtered, velocity_angular_filtered, last_pitch
    global Ki, integral_error
    
    # note: last_pitch is already corrected for yaw heuristically
    pitch = last_pitch
    pitch_dot = pitch_dot_filtered

    # Standard LQR control
    velocity_linear_error = velocity_linear_set_point - (velocity_angular_filtered * WHEEL_RADIUS)
    lqr_v = LQR_K[0] * (0 - pitch) + LQR_K[1] * pitch_dot + LQR_K[2] * 0 + LQR_K[3] * velocity_linear_error
    
    # Add integral term to reduce steady-state error and wobbling
    integral_contribution = Ki * integral_error
    
    total_control = lqr_v + integral_contribution
    
    return -total_control / WHEEL_RADIUS