import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

def plot_results(csv_path):
    # check if file exists
    if not os.path.exists(csv_path):
        print(f"Error: Could not find '{csv_path}'")
        print("Make sure you have run the rust test first: cargo test --test controller_test")
        sys.exit(1)

    # Read the CSV data
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    # Set up the plot style
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # Create a figure with a 3x2 grid
    # Left column: Rates (Commanded vs Actual)
    # Right column: Torques (Control Output)
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    fig.suptitle(f'Controller Simulation Results\nSource: {csv_path}', fontsize=16)

    # Time vector
    t = df['time_s']

    # --- Roll (Row 0) ---
    # Rates
    ax_roll = axes[0, 0]
    ax_roll.plot(t, df['cmd_roll_rad_s'], 'r--', label='Commanded', alpha=0.8)
    ax_roll.plot(t, df['act_roll_rad_s'], 'b-', label='Actual', linewidth=1.5)
    ax_roll.set_title('Roll Rate (p)')
    ax_roll.set_ylabel('Rate (rad/s)')
    ax_roll.legend(loc='upper right')
    ax_roll.grid(True)

    # Torque
    ax_tq_x = axes[0, 1]
    ax_tq_x.plot(t, df['torque_x'], 'g-', label='Torque X')
    ax_tq_x.set_title('Roll Torque Output')
    ax_tq_x.set_ylabel('Torque (N*m)')
    ax_tq_x.grid(True)

    # --- Pitch (Row 1) ---
    # Rates
    ax_pitch = axes[1, 0]
    ax_pitch.plot(t, df['cmd_pitch_rad_s'], 'r--', label='Commanded', alpha=0.8)
    ax_pitch.plot(t, df['act_pitch_rad_s'], 'b-', label='Actual', linewidth=1.5)
    ax_pitch.set_title('Pitch Rate (q)')
    ax_pitch.set_ylabel('Rate (rad/s)')
    ax_pitch.grid(True)

    # Torque
    ax_tq_y = axes[1, 1]
    ax_tq_y.plot(t, df['torque_y'], 'g-', label='Torque Y')
    ax_tq_y.set_title('Pitch Torque Output')
    ax_tq_y.set_ylabel('Torque (N*m)')
    ax_tq_y.grid(True)

    # --- Yaw (Row 2) ---
    # Rates
    ax_yaw = axes[2, 0]
    ax_yaw.plot(t, df['cmd_yaw_rad_s'], 'r--', label='Commanded', alpha=0.8)
    ax_yaw.plot(t, df['act_yaw_rad_s'], 'b-', label='Actual', linewidth=1.5)
    ax_yaw.set_title('Yaw Rate (r)')
    ax_yaw.set_ylabel('Rate (rad/s)')
    ax_yaw.set_xlabel('Time (s)')
    ax_yaw.grid(True)

    # Torque
    ax_tq_z = axes[2, 1]
    ax_tq_z.plot(t, df['torque_z'], 'g-', label='Torque Z')
    ax_tq_z.set_title('Yaw Torque Output')
    ax_tq_z.set_ylabel('Torque (N*m)')
    ax_tq_z.set_xlabel('Time (s)')
    ax_tq_z.grid(True)

    plt.tight_layout()
    
    # Save the plot
    output_image = csv_path.replace('.csv', '.png')
    plt.savefig(output_image, dpi=150)
    print(f"Plot saved to: {output_image}")
    
    # Show the plot
    plt.show()

if __name__ == "__main__":
    # Default path assumes script is run from crate root or tests dir
    default_path = "./rust_controller_results.csv"
    
    # Allow command line argument override
    path = sys.argv[1] if len(sys.argv) > 1 else default_path
    
    # Handle running from crate root vs inside tests/
    if not os.path.exists(path) and os.path.exists(f"../{path}"):
        path = f"../{path}"
        
    plot_results(path)