import numpy as np

def compute(PID, setpoint, measurement, dt, max_integral=np.inf):
    kp = PID['P']
    ki = PID['I']
    kd = PID['D']

    integral = 0.0
    prev_error = 0.0
    max_integral = max_integral

    error = setpoint - measurement
    integral += error * dt
    integral = np.clip(integral, -max_integral, max_integral)
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    prev_error = error
    return kp * error + ki * integral + kd * derivative


# -----------------------------------
# PID controller parameters
# -----------------------------------
# Layer 1: Position → Velocity
pid_pos_x = {'P': 1.2, 'I': 0.0, 'D': 0.4}
pid_pos_y = {'P': 1.2, 'I': 0.0, 'D': 0.4}
pid_pos_z = {'P': 2.0, 'I': 0.1, 'D': 0.5}

# Layer 2: Velocity → Attitude (pitch/roll)
pid_vel_x = {'P': 0.5, 'I': 0.0, 'D': 0.2}
pid_vel_y = {'P': 0.5, 'I': 0.0, 'D': 0.2}
pid_vel_z = {'P': 1.0, 'I': 0.05, 'D': 0.3}  # for thrust estimation (not returned)

# Layer 3: Attitude → Angular rates
pid_roll = {'P': 3.0, 'I': 0.0, 'D': 0.1}
pid_pitch = {'P': 3.0, 'I': 0.0, 'D': 0.1}
pid_yaw = {'P': 3.0, 'I': 0.1, 'D': 0.2}

# -----------------------------------
# Implement a controller (3-layer PID)
# -----------------------------------
def controller(state, target_pos, dt):
    # state format: [x, y, z, roll, pitch, yaw]
    # target_pos format: (x, y, z, yaw)
    # dt: time step (s)
    # return velocity command format: (vx_setpoint, vy_setpoint, vz_setpoint, yaw_rate_setpoint)

    x, y, z, roll, pitch, yaw = state
    tx, ty, tz, target_yaw = target_pos

    # Layer 1: Position → Velocity setpoints
    vx_sp = compute(pid_pos_x, tx, x, dt) 
    vy_sp = compute(pid_pos_y, ty, y, dt)
    vz_sp = compute(pid_pos_z, tz, z, dt)

    # Let's assume measured velocities (in body or world frame) are zero for simplicity
    # In a real system, you'd pass the actual velocities from sensors here
    vx_meas, vy_meas, vz_meas = 0.0, 0.0, 0.0

    # Layer 2: Velocity → Attitude setpoints
    pitch_sp = compute(pid_vel_x, vx_sp, vx_meas, dt)  # pitch controls x velocity
    roll_sp  = compute(pid_vel_y, vy_sp, vy_meas, dt)  # roll controls y velocity (negative for right-hand system)
    thrust   = compute(pid_vel_z, vz_sp, vz_meas, dt)   # thrust output

    # Layer 3: Attitude → Angular rates
    roll_rate_sp  = compute(pid_roll, roll_sp, roll, dt)
    pitch_rate_sp = compute(pid_pitch, pitch_sp, pitch, dt)

    # Yaw control: compute rate to correct yaw
    yaw_error = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
    yaw_rate_sp = compute(pid_yaw, 0.0, -yaw_error, dt)

    # Final control output: velocity setpoints and yaw rate
    output = (vx_sp, vy_sp, vz_sp, yaw_rate_sp)
    return output


