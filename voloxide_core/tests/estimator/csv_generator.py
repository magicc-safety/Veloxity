# import numpy as np
# import pandas as pd
#
# # --- Quaternion utility funcs ---
# def quat_prod(q1, q2):
#     w1, x1, y1, z1 = q1; w2, x2, y2, z2 = q2
#     w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#     x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
#     y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
#     z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
#     return np.array([w, x, y, z], dtype=float)
#
# def quat_conjugate(q):
#     return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)
#
# def quat_normalize(q):
#     n = np.linalg.norm(q)
#     return q / n if n != 0 else np.array([1.0, 0.0, 0.0, 0.0])
#
# # --- Sensor spoofing ---
# def spoof_accelerometer(q_true):
#     g_inertial_q = np.array([0.0, 0.0, 0.0, 1.0])
#     q_conj = quat_conjugate(q_true)
#     gravity_in_body_q = quat_prod(quat_prod(q_conj, g_inertial_q), q_true)
#     accel = -gravity_in_body_q[1:]
#     n = np.linalg.norm(accel)
#     return accel / n if n != 0 else np.array([0.0, 0.0, 1.0])
#
# # --- Data Generation ---
# def generate_data_and_save():
#     dt = 1.0 / 400.0; sim_time = 20.0
#     t_span = np.arange(0.0, sim_time, dt); N = len(t_span)
#
#     true_bias = np.array([0.05, -0.05, 0.02]); true_quat = np.zeros((N, 4))
#     true_quat[0] = [1.0, 0.0, 0.0, 0.0]
#
#     rng = np.random.default_rng(0); noise_state = np.zeros(3)
#     band_alpha=0.98; gyro_process_sigma=0.12; gyro_meas_sigma=0.03; bias_walk_sigma=0.001
#
#     data_log = []
#     for i in range(N):
#         true_omega = np.zeros(3)
#         if i > 0:
#             wx = 0.35 * np.sin(t_span[i]*0.8); wy = 0.25 * np.cos(t_span[i]*0.5)
#             wz = 0.08 * np.sin(t_span[i]*1.2) + 0.02 * rng.normal()
#             true_omega_det = np.array([wx, wy, wz])
#             noise_state = band_alpha*noise_state + np.sqrt(1-band_alpha**2)*gyro_process_sigma*rng.normal(size=3)
#             true_omega = true_omega_det + noise_state
#             true_bias += bias_walk_sigma * rng.normal(size=3) * np.sqrt(dt)
#             q_prev = true_quat[i-1]
#             q_dot = 0.5 * quat_prod(q_prev, np.concatenate(([0.0], true_omega)))
#             true_quat[i] = quat_normalize(q_prev + q_dot * dt)
#
#         omega_y = true_omega + true_bias + gyro_meas_sigma * rng.normal(size=3)
#         v_a = spoof_accelerometer(true_quat[i])
#
#         row = {
#             'time': t_span[i],
#             'accel_x': v_a[0], 'accel_y': v_a[1], 'accel_z': v_a[2],
#             'gyro_x': omega_y[0], 'gyro_y': omega_y[1], 'gyro_z': omega_y[2],
#             'true_quat_w': true_quat[i][0], 'true_quat_x': true_quat[i][1],
#             'true_quat_y': true_quat[i][2], 'true_quat_z': true_quat[i][3],
#             'true_bias_x': true_bias[0], 'true_bias_y': true_bias[1], 'true_bias_z': true_bias[2],
#             'true_omega_x': true_omega[0], 'true_omega_y': true_omega[1], 'true_omega_z': true_omega[2],
#         }
#         data_log.append(row)
#
#     df = pd.DataFrame(data_log)
#     output_filename = '/Users/workhorse/Coding/rust/voloxide/voloxide_core/tests/imu_sensor_data.csv'
#     df.to_csv(output_filename, index=False)
#     print(f"Successfully generated full data to {output_filename}")
#
# if __name__ == "__main__":
#     generate_data_and_save()




































import numpy as np
import pandas as pd

# --- Quaternion utility funcs ---
def quat_prod(q1, q2):
    w1, x1, y1, z1 = q1; w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z], dtype=float)

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n != 0 else np.array([1.0, 0.0, 0.0, 0.0])

# --- Sensor spoofing ---
def spoof_accelerometer(q_true):
    g_inertial_q = np.array([0.0, 0.0, 0.0, 1.0])
    q_conj = quat_conjugate(q_true)
    gravity_in_body_q = quat_prod(quat_prod(q_conj, g_inertial_q), q_true)
    accel = -gravity_in_body_q[1:]
    n = np.linalg.norm(accel)
    return accel / n if n != 0 else np.array([0.0, 0.0, 1.0])

# --- Data Generation ---
def generate_data_and_save():
    """
    Generates simulated IMU data and saves it to a CSV for the Rust test.
    The "true bias" saved is the constant initial value, matching the plotting
    logic in the Python-only simulation.
    """
    # Simulation parameters
    dt = 1.0 / 400.0
    sim_time = 100.0
    t_span = np.arange(0.0, sim_time, dt)
    N = len(t_span)
    
    # True state initialization
    true_bias_initial = np.array([0.05, -0.05, 0.02])
    true_bias_timevarying = true_bias_initial.copy() # This bias will drift
    true_quat = np.zeros((N, 4))
    true_quat[0] = [1.0, 0.0, 0.0, 0.0]
    
    # Noise parametersa
    rng = np.random.default_rng(0)
    noise_state = np.zeros(3)
    band_alpha=0.98
    gyro_process_sigma=0.12
    gyro_meas_sigma=0.03
    bias_walk_sigma=0.01
    
    data_log = []
    # Main simulation loop
    for i in range(N):
        true_omega = np.zeros(3)
        if i > 0:
            # Generate deterministic angular velocity
            wx = 0.35 * np.sin(t_span[i]*0.8)
            wy = 0.25 * np.cos(t_span[i]*0.5)
            wz = 0.08 * np.sin(t_span[i]*1.2) + 0.02 * rng.normal()
            true_omega_det = np.array([wx, wy, wz])

            # Add colored process noise
            noise_state = band_alpha*noise_state + np.sqrt(1-band_alpha**2)*gyro_process_sigma*rng.normal(size=3)
            true_omega = true_omega_det + noise_state

            # Bias random walk (using the time-varying bias)
            # Scaled by dt to match the reference simulation exactly
            true_bias_timevarying += bias_walk_sigma * rng.normal(size=3) * dt 
            
            # Integrate true quaternion
            q_prev = true_quat[i-1]
            q_dot = 0.5 * quat_prod(q_prev, np.concatenate(([0.0], true_omega)))
            true_quat[i] = quat_normalize(q_prev + q_dot * dt)

        # Generate sensor measurements using the time-varying bias
        omega_y = true_omega + true_bias_timevarying + gyro_meas_sigma * rng.normal(size=3)
        v_a = spoof_accelerometer(true_quat[i])

        # Log data for this timestep
        row = {
            'time': t_span[i],
            'accel_x': v_a[0], 'accel_y': v_a[1], 'accel_z': v_a[2],
            'gyro_x': omega_y[0], 'gyro_y': omega_y[1], 'gyro_z': omega_y[2],
            'true_quat_w': true_quat[i][0], 'true_quat_x': true_quat[i][1],
            'true_quat_y': true_quat[i][2], 'true_quat_z': true_quat[i][3],
            # Save the CONSTANT INITIAL bias as the ground truth for plotting
            'true_bias_x': true_bias_initial[0], 
            'true_bias_y': true_bias_initial[1], 
            'true_bias_z': true_bias_initial[2],
            'true_omega_x': true_omega[0], 'true_omega_y': true_omega[1], 'true_omega_z': true_omega[2],
        }
        data_log.append(row)

    # --- Save to CSV ---
    df = pd.DataFrame(data_log)
    output_filename = './tests/estimator/imu_sensor_data.csv'
    df.to_csv(output_filename, index=False)
    print(f"Successfully generated full data to {output_filename}")

if __name__ == "__main__":
    generate_data_and_save()
