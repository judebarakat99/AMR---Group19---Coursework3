
# Implement a controller

# PID Controller for drone position and yaw
# No external libraries are used except built-in ones from the script

# PID Gains (Tune these!)
Kp_pos = [0.45, 0.5, 0.45]  # Proportional gain for (x, y, z)
Ki_pos = [0.001, 0.001, 0.001]  # Integral gain
Kd_pos = [0.08, 0.08, 0.08]  # Derivative gain

Kp_yaw = 0.3  # Yaw proportional gain
Ki_yaw = 0.001  # Yaw integral gain
Kd_yaw = 0.03  # Yaw derivative gain

# Previous errors (for derivative term)
prev_error_pos = [0.0, 0.0, 0.0]
integral_error_pos = [0.0, 0.0, 0.0]

prev_error_yaw = 0.0
integral_error_yaw = 0.0

def controller(state, target, dt):
    global prev_error_pos, integral_error_pos, prev_error_yaw, integral_error_yaw

    # Extract current position and yaw from state
    pos = [state[0], state[1], state[2]]  # (x, y, z)
    yaw = state[5]  # Yaw angle

    # Extract target position and yaw
    target_pos = [target[0], target[1], target[2]]
    target_yaw = target[3]

    # Compute errors
    error_pos = [target_pos[i] - pos[i] for i in range(3)]
    error_yaw = ((target_yaw - yaw + 3.14159) % (2 * 3.14159)) - 3.14159  # wrap [-pi, pi]

    # Update integral and derivative terms
    derivative_pos = [(error_pos[i] - prev_error_pos[i]) / dt for i in range(3)]
    for i in range(3):
        integral_error_pos[i] += error_pos[i] * dt

    derivative_yaw = (error_yaw - prev_error_yaw) / dt
    integral_error_yaw += error_yaw * dt

    # Compute velocity commands using PID
    velocity = [
        Kp_pos[i] * error_pos[i] +
        Ki_pos[i] * integral_error_pos[i] +
        Kd_pos[i] * derivative_pos[i]
        for i in range(3)
    ]

    yaw_rate = (
        Kp_yaw * error_yaw +
        Ki_yaw * integral_error_yaw +
        Kd_yaw * derivative_yaw
    )

    # Save current errors for next step
    prev_error_pos = error_pos[:]
    prev_error_yaw = error_yaw

    return velocity + [yaw_rate]