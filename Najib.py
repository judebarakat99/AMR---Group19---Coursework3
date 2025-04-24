# Implement a controller

# PID Controller for drone position and yaw
# No external libraries are used except built-in ones from the script

# PID Gains (Tune these!)
Kp_pos = [0.45, 0.5, 0.45]  # Proportional gain for (x, y, z)
Ki_pos = [0.001, 0.001, 0.001]  # Integral gain
Kd_pos = [0.08, 0.08, 0.08]  # Derivative gain

Kp_yaw = 0.3 # Yaw proportional gain
Ki_yaw = 0.001   # Yaw integral gain
Kd_yaw = 0.03   # Yaw derivative gain

# Previous errors (for derivative term)
prev_error_pos = [0.0, 0.0, 0.0]
integral_error_pos = [0.0, 0.0, 0.0]

prev_error_yaw = 0.0
integral_error_yaw = 0.0

def controller(state, target, dt):
    global prev_error_pos, integral_error_pos, prev_error_yaw, integral_error_yaw

    # Extract current position and yaw from state
    pos = [state[0], state[1], state[2]]  # (x, y, z)
    yaw = state[5]  # Yaw angle (assumed last in state)

    # Extract target position and yaw
    target_pos = [target[0], target[1], target[2]]
    target_yaw = target[3]

    # Compute position errors
    error_pos = [target_pos[i] - pos[i] for i in range(3)]
    
    # Update integral term
    for i in range(3):
        integral_error_pos[i] += error_pos[i] * dt
    
    # Compute derivative term
    derivative_error_pos = [(error_pos[i] - prev_error_pos[i]) / dt if dt > 0 else 0 for i in range(3)]
    
    # Store previous error
    prev_error_pos = error_pos

    # Compute PID output for velocity
    desired_vel = [
        Kp_pos[i] * error_pos[i] + Ki_pos[i] * integral_error_pos[i] + Kd_pos[i] * derivative_error_pos[i]
        for i in range(3)
    ]

    # Compute yaw error
    yaw_error = target_yaw - yaw
    integral_error_yaw += yaw_error * dt
    derivative_error_yaw = (yaw_error - prev_error_yaw) / dt if dt > 0 else 0
    prev_error_yaw = yaw_error

    # Compute PID output for yaw rate
    yaw_rate = (Kp_yaw * yaw_error) + (Ki_yaw * integral_error_yaw) + (Kd_yaw * derivative_error_yaw)

    # Limit velocity values (for safety)
    max_vel = 1.0
    max_yaw_rate = 1.0
    desired_vel = [max(-max_vel, min(v, max_vel)) for v in desired_vel]
    yaw_rate = max(-max_yaw_rate, min(yaw_rate, max_yaw_rate))

    return (desired_vel[0], desired_vel[1], desired_vel[2], yaw_rate)
