import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# --- Color Scheme ---
neon_sorbet = {
    "Electric Teal": "#00DEDE",
    "Hot Pink": "#FF69B4",
    "Zesty Lime": "#89CA26",
    "Solar Flare Orange": "#FF4500",
    "Cyber Grape": "#9932CC",
}

# --- Quaternion utility func (for plotting) ---
def quat_to_euler(q):
    """Converts a quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in radians."""
    w, x, y, z = q
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])

def plot_rust_results(filename='voloxide_estimator_results.csv'):
    """Reads the CSV output from the Rust test and generates comparison plots."""
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: Could not find '{filename}'.")
        print("Please run 'cargo test' to generate the results file first.")
        return

    # --- Convert truth quaternions to Euler angles ---
    true_quat_cols = ['true_quat_w', 'true_quat_x', 'true_quat_y', 'true_quat_z']
    true_euler_rad = np.array([quat_to_euler(q) for q in df[true_quat_cols].values])
    true_euler_deg = np.rad2deg(true_euler_rad)

    # --- Convert estimated angles to degrees ---
    est_euler_deg = np.rad2deg(df[['est_roll_rad', 'est_pitch_rad', 'est_yaw_rad']])

    # --- PLOT 1: Attitude Estimation ---
    fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Rust Mahony Filter: Attitude Estimation', fontsize=16)
    labels = ['Roll', 'Pitch', 'Yaw']
    attitude_colors = [neon_sorbet["Electric Teal"], neon_sorbet["Zesty Lime"], neon_sorbet["Hot Pink"]]
    for i in range(3):
        axs[i].plot(df['time'], true_euler_deg[:, i], 'k-', linewidth=1.5, label=f'True {labels[i]}')
        axs[i].plot(df['time'], est_euler_deg.iloc[:, i], '-', linewidth=2, color=attitude_colors[i], label=f'Rust Est. {labels[i]}')
        axs[i].set_ylabel(f'{labels[i]} (deg)')
        axs[i].legend(); axs[i].grid(True)
    axs[2].set_xlabel('Time (s)')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # --- PLOT 2: Gyro Bias Estimation ---
    fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Rust Mahony Filter: Gyro Bias Estimation', fontsize=16)
    bias_labels = ['X', 'Y', 'Z']
    bias_colors = [neon_sorbet["Electric Teal"], neon_sorbet["Zesty Lime"], neon_sorbet["Hot Pink"]]
    for i, axis in enumerate(bias_labels):
        axs[i].plot(df['time'], df[f'true_bias_{axis.lower()}'], 'k-', label=f'True Bias {axis}')
        axs[i].plot(df['time'], df[f'est_bias_{axis.lower()}'], '-', color=bias_colors[i], label=f'Rust Est. Bias {axis}')
        axs[i].set_ylabel(f'Bias {axis} (rad/s)')
        axs[i].legend(); axs[i].grid(True)
    axs[2].set_xlabel('Time (s)')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # --- PLOT 3: Gyro Measurements ---
    fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('True vs. Measured Angular Velocity', fontsize=16)
    omega_labels = ['X', 'Y', 'Z']
    omega_colors = [neon_sorbet["Electric Teal"], neon_sorbet["Zesty Lime"], neon_sorbet["Hot Pink"]]
    for i, axis in enumerate(omega_labels):
        axs[i].plot(df['time'], df[f'true_omega_{axis.lower()}'], 'k-', linewidth=1, label=f'True ω_{axis.lower()}')
        axs[i].plot(df['time'], df[f'meas_gyro_{axis.lower()}'], '-', linewidth=1.5, alpha=0.8, color=omega_colors[i], label=f'Measured ω_{axis.lower()}')
        axs[i].set_ylabel(f'ω_{axis.lower()} (rad/s)')
        axs[i].legend(); axs[i].grid(True)
    axs[2].set_xlabel('Time (s)')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save figure
    output_image = filename.replace('.csv', '.png')
    plt.savefig(output_image, dpi=150)
    print(f"Plot saved to: {output_image}")

    plt.show()

if __name__ == "__main__":
    # Assumes the Rust output file is in the same directory or a known path
    plot_rust_results()
