# mahony_simulation.py
import numpy as np
import matplotlib.pyplot as plt
# plt.style.use('dark_background')

neon_sorbet = {
    "Electric Teal": "#00DEDE",
    "Hot Pink": "#FF69B4",
    "Zesty Lime": "#89CA26",
    "Solar Flare Orange": "#FF4500",
    "Cyber Grape": "#9932CC",
}

# -------------------------
# Quaternion utility funcs
# -------------------------


def quat_prod(q1, q2):
    # Hamilton product q1 ⊗ q2, q = [w, x, y, z]
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z], dtype=float)


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_norm(q):
    return np.linalg.norm(q)


def quat_normalize(q):
    n = quat_norm(q)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_inverse(q):
    # general inverse (works even if not exactly unit)
    n2 = np.dot(q, q)
    if n2 == 0:
        return quat_conjugate(q)
    return quat_conjugate(q) / n2


def quat_to_euler(q):
    # returns [roll, pitch, yaw] in radians
    w, x, y, z = q
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)
    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)


# -------------------------
# Sensor spoofing
# -------------------------


def spoof_accelerometer(q_true):
    """
    Given true quaternion q_true (body -> inertial), return accelerometer
    measurement vector in body frame, normalized. Assumes only gravity (no linear acc).
    Accelerometer measures reaction force opposite gravity, so return -g_body (unit).
    """
    # inertial gravity pointing +z (0,0,1)
    g_inertial_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # pure quaternion
    q_conj = quat_conjugate(q_true)
    temp = quat_prod(q_conj, g_inertial_q)  # q_conj ⊗ g ⊗ q
    gravity_in_body_q = quat_prod(temp, q_true)
    g_body = gravity_in_body_q[1:]  # vector part
    # accelerometer measures reaction = -g_body (assuming no other accel)
    accel = -g_body
    n = np.linalg.norm(accel)
    if n == 0:
        return np.array([0.0, 0.0, 1.0])
    return accel / n


# -------------------------
# Mahony filter (vector-error implementation)
# -------------------------


class MahonyFilter:
    def __init__(self, k_p=1.0, k_i=0.0, q_initial=None):
        self.k_p = float(k_p)
        self.k_i = float(k_i)
        if q_initial is None:
            q_initial = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.q_hat = quat_normalize(np.array(q_initial, dtype=float))
        self.b_hat = np.zeros(3, dtype=float)  # gyro bias estimate

    def update(self, omega_y, v_a, dt):
        # normalize accelerometer measurement
        v_a = np.array(v_a, dtype=float)
        na = np.linalg.norm(v_a)
        if na == 0 or dt <= 0:
            return self.q_hat
        v_a = v_a / na

        # predicted gravity in body frame using q_hat: v_hat = q_conj ⊗ g_inertial ⊗ q
        g_inertial_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        q_conj = quat_conjugate(self.q_hat)
        tmp = quat_prod(q_conj, g_inertial_q)
        gravity_in_body_q = quat_prod(tmp, self.q_hat)
        v_hat = gravity_in_body_q[1:]

        # vector error (predicted × measured)
        e = np.cross(v_hat, v_a)

        # integral (bias) update
        b_dot = -self.k_i * e
        self.b_hat += b_dot * dt

        # corrected angular rate (body frame)
        # If signs are opposite in your convention, change +self.k_p*e to -self.k_p*e
        omega_corr = omega_y - self.b_hat + self.k_p * e

        # quaternion derivative
        omega_q = np.concatenate(([0.0], omega_corr))
        q_dot = 0.5 * quat_prod(self.q_hat, omega_q)

        # integrate and normalize
        self.q_hat = quat_normalize(self.q_hat + q_dot * dt)

        return self.q_hat


# -------------------------
# Simulation
# -------------------------


def run_simulation():
    # Simulation parameters
    dt = 0.0025  # 400 hz
    sim_time = 100.0  # seconds
    t_span = np.arange(0.0, sim_time, dt)
    N = len(t_span)

    # True state
    true_bias0 = np.array([0.05, -0.05, 0.02], dtype=float)
    true_bias_time = true_bias0.copy()
    true_quat = np.zeros((N, 4), dtype=float)
    true_quat[0] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    # Noisy motion parameters (colored noise + bias walk)
    rng = np.random.default_rng(0)  # reproducible
    noise_state = np.zeros(3, dtype=float)
    band_alpha = 0.98
    # band_alpha = 0.5
    gyro_process_sigma = 0.12
    # gyro_process_sigma = 0.5
    gyro_meas_sigma = 0.03
    # gyro_meas_sigma = 0.1
    bias_walk_sigma = 0.01
    # bias_walk_sigma = 0.1

    # Filter setup
    mahony = MahonyFilter(k_p=1.5, k_i=0.05, q_initial=true_quat[0].copy())

    # Storage for estimates and measurements
    est_quat = np.zeros_like(true_quat)
    est_quat[0] = mahony.q_hat.copy()
    est_bias = np.zeros((N, 3), dtype=float)
    est_bias[0] = mahony.b_hat.copy()

    true_omega_storage = np.zeros((N, 3), dtype=float)
    meas_omega_storage = np.zeros((N, 3), dtype=float)

    # Main loop
    for i in range(1, N):
        # deterministic angular velocity (larger / more interesting)
        wx = 0.35 * np.sin(t_span[i] * 0.8)
        wy = 0.25 * np.cos(t_span[i] * 0.5)
        wz = 0.08 * np.sin(t_span[i] * 1.2) + 0.02 * rng.normal()  # small jitter
        true_omega_det = np.array([wx, wy, wz], dtype=float)

        # colored process noise (first-order low-pass)
        noise_state = band_alpha * noise_state + np.sqrt(
            1.0 - band_alpha**2
        ) * gyro_process_sigma * rng.normal(size=3)

        # true instantaneous angular velocity (deterministic + colored)
        true_omega = true_omega_det + noise_state
        true_omega_storage[i] = true_omega

        # bias random walk (integrate small increments scaled by dt)
        true_bias_time += bias_walk_sigma * rng.normal(size=3) * dt

        # measurement (what filter receives): gyro + bias + white noise
        gyro_meas_noise = gyro_meas_sigma * rng.normal(size=3)
        omega_y = true_omega + true_bias_time + gyro_meas_noise
        meas_omega_storage[i] = omega_y

        # integrate true quaternion from true_omega (body frame)
        q_prev = true_quat[i - 1]
        q_dot_true = 0.5 * quat_prod(q_prev, np.concatenate(([0.0], true_omega)))
        true_quat[i] = quat_normalize(q_prev + q_dot_true * dt)

        # accelerometer measurement from true attitude (body frame)
        v_a = spoof_accelerometer(true_quat[i])

        # update filter with measured gyro and accel
        q_est = mahony.update(omega_y, v_a, dt)

        # store results
        est_quat[i] = q_est
        est_bias[i] = mahony.b_hat.copy()

    # Convert to Euler for plotting (degrees)
    true_euler = np.array([quat_to_euler(q) for q in true_quat])
    est_euler = np.array([quat_to_euler(q) for q in est_quat])
    true_euler_deg = np.rad2deg(true_euler)
    est_euler_deg = np.rad2deg(est_euler)

    # PLOTS
    plt.figure(figsize=(12, 9))
    plt.suptitle("Mahony Filter: Attitude Estimation (noisy truth)", fontsize=14)

    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(t_span, true_euler_deg[:, 0], "k-", linewidth=1.0, label="True Roll")
    ax1.plot(
        t_span,
        est_euler_deg[:, 0],
        color=neon_sorbet["Electric Teal"],
        linestyle="-",
        linewidth=2.0,
        # marker='o',
        # markersize=1.1,
        # markevery=10,
        # markeredgecolor=stone_and_sage['Deep Slate'],
        # alpha=0.5,
        label="Estimated Roll",
    )
    ax1.set_ylabel("Roll (deg)")
    ax1.legend()
    ax1.grid(True)

    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(t_span, true_euler_deg[:, 1], "k-", linewidth=1.0, label="True Pitch")
    ax2.plot(
        t_span,
        est_euler_deg[:, 1],
        color=neon_sorbet["Zesty Lime"],
        linestyle="-",
        linewidth=2.0,
        # marker='o',
        # markersize=1.1,
        # markevery=10,
        # markeredgecolor=stone_and_sage['Deep Slate'],
        # alpha=0.5,
        label="Estimated Roll",
    )
    ax2.set_ylabel("Pitch (deg)")
    ax2.legend()
    ax2.grid(True)

    ax3 = plt.subplot(3, 1, 3)
    ax3.plot(t_span, true_euler_deg[:, 2], "k-", linewidth=1.0, label="True Yaw")
    ax3.plot(
        t_span,
        est_euler_deg[:, 2],
        color=neon_sorbet["Hot Pink"],
        linestyle="-",
        linewidth=2.0,
        # marker='o',
        # markersize=1.1,
        # markevery=10,
        # markeredgecolor=stone_and_sage['Deep Slate'],
        # alpha=0.5,
        label="Estimated Roll",
    )
    ax3.set_ylabel("Yaw (deg)")
    ax3.set_xlabel("Time (s)")
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout(rect=(0, 0.03, 1, 0.95))

    # Bias plots
    plt.figure(figsize=(10, 7))
    plt.suptitle("Gyro Bias Estimation", fontsize=14)
    plot_colors = [
        neon_sorbet["Electric Teal"],
        neon_sorbet["Zesty Lime"],
        neon_sorbet["Hot Pink"],
    ]
    labels = ["X", "Y", "Z"]
    for k in range(3):
        plt.subplot(3, 1, k + 1)
        plt.plot(
            t_span,
            np.full_like(t_span, true_bias0[k]),
            "k-",
            label=f"True init bias {labels[k]} (start)",
        )
        plt.plot(
            t_span,
            est_bias[:, k],
            "--",
            label=f"Estimated bias {labels[k]}",
            color=plot_colors[k],
        )
        plt.ylabel(f"Bias {labels[k]} (rad/s)")
        plt.legend()
        plt.grid(True)
    plt.xlabel("Time (s)")
    plt.tight_layout(rect=(0, 0.03, 1, 0.95))

    # Compare true vs measured omega for a short window (tiled subplots)
    window = slice(0, int(200.0 / dt))  # first 3 seconds
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("True ω vs Measured ω", fontsize=14)

    axs[0].plot(
        t_span[window],
        true_omega_storage[window, 0],
        label="true wx",
        linewidth=0.5,
        color="black",
    )
    axs[0].plot(
        t_span[window],
        meas_omega_storage[window, 0],
        "-",
        label="meas wx",
        linewidth=0.5,
        color=neon_sorbet["Electric Teal"],
    )
    axs[0].set_ylabel("ω_x (rad/s)")
    axs[0].legend(loc="upper right")
    axs[0].grid(True)

    axs[1].plot(
        t_span[window],
        true_omega_storage[window, 1],
        label="true wy",
        linewidth=0.5,
        color="black",
    )
    axs[1].plot(
        t_span[window],
        meas_omega_storage[window, 1],
        "-",
        label="meas wy",
        linewidth=0.5,
        color=neon_sorbet["Zesty Lime"],
    )
    axs[1].set_ylabel("ω_y (rad/s)")
    axs[1].legend(loc="upper right")
    axs[1].grid(True)

    axs[2].plot(
        t_span[window],
        true_omega_storage[window, 2],
        label="true wz",
        linewidth=0.5,
        color="black",
    )
    axs[2].plot(
        t_span[window],
        meas_omega_storage[window, 2],
        "-",
        label="meas wz",
        linewidth=0.5,
        color=neon_sorbet["Hot Pink"],
    )
    axs[2].set_ylabel("ω_z (rad/s)")
    axs[2].set_xlabel("Time (s)")
    axs[2].legend(loc="upper right")
    axs[2].grid(True)

    plt.tight_layout(rect=(0, 0.03, 1, 0.95))

    plt.show()


if __name__ == "__main__":
    run_simulation()
