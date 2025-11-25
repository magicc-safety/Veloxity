# import pandas as pd
# import matplotlib.pyplot as plt
# import os
# import sys

# # --- Color Scheme ---
# neon_sorbet = {
#     "Electric Teal": "#00DEDE",
#     "Hot Pink": "#FF69B4",
#     "Zesty Lime": "#89CA26",
#     "Solar Flare Orange": "#FF4500",
#     "Cyber Grape": "#9932CC",
# }

# def plot_results(csv_path):
#     # Verify file existence
#     if not os.path.exists(csv_path):
#         print(f"Error: Could not find '{csv_path}'")
#         sys.exit(1)

#     # Load CSV data
#     try:
#         df = pd.read_csv(csv_path)
#     except Exception as e:
#         print(f"Error reading CSV: {e}")
#         sys.exit(1)

#     # Setup plot aesthetics
#     plt.style.use('seaborn-v0_8-darkgrid')
    
#     # Create 3x2 grid
#     # Left Column: Tracking (Command vs Actual)
#     # Right Column: Control Effort (Torque)
#     fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
#     fig.suptitle(f'Controller Simulation Results\nSwitching: Rate Mode -> Angle Mode', fontsize=16)

#     t = df['time_s']
#     for ax in axes.flatten():
#         ax.set_xlim(0, 10)
    
#     # Helper to shade the background where Angle Mode is active
#     # Fixed to handle multiple non-contiguous blocks of Angle mode
#     def shade_angle_mode(ax):
#         is_angle = df['mode_id'] == 1
        
#         # Group consecutive rows with the same mode to identify contiguous blocks
#         # (df['mode_id'] != df['mode_id'].shift()).cumsum() creates a new group ID every time the mode changes
#         groups = (df['mode_id'] != df['mode_id'].shift()).cumsum()
        
#         # Iterate only over the groups that are actually Angle mode
#         for _, group in df[is_angle].groupby(groups):
#             start_t = group['time_s'].iloc[0]
#             end_t = group['time_s'].iloc[-1]
#             # Add the shaded span for this specific block
#             ax.axvspan(start_t, end_t, color='gray', alpha=0.15, lw=0)

#     # Helper to plot Rate/Angle split
#     def plot_dual_mode(ax, time, mode_series, cmd_series, rate_act, angle_act, title_prefix):
#         mask_rate = mode_series == 0
#         mask_angle = mode_series == 1
        
#         # Rate Section (Electric Teal / Hot Pink)
#         if mask_rate.any():
#             ax.plot(time[mask_rate], rate_act[mask_rate], color=neon_sorbet["Electric Teal"], linewidth=1.5, label='Actual Rate (rad/s)')
#             ax.plot(time[mask_rate], cmd_series[mask_rate], color=neon_sorbet["Hot Pink"], linestyle='--', linewidth=2, label='Cmd Rate (rad/s)')
            
#         # Angle Section (Zesty Lime / Solar Flare Orange)
#         if mask_angle.any():
#             ax.plot(time[mask_angle], angle_act[mask_angle], color=neon_sorbet["Zesty Lime"], linewidth=1.5, label='Actual Angle (rad)')
#             ax.plot(time[mask_angle], cmd_series[mask_angle], color=neon_sorbet["Solar Flare Orange"], linestyle='--', linewidth=2, label='Cmd Angle (rad)')
            
#         shade_angle_mode(ax)
#         ax.set_title(f'{title_prefix}: Tracking')
#         ax.legend(loc='upper left', fontsize='small')
#         ax.grid(True)

#     # --- Row 0: Roll ---
#     plot_dual_mode(
#         axes[0, 0], t, df['mode_id'], df['cmd_x'], 
#         df['act_p_rad_s'], df['act_roll_rad'], "Roll"
#     )
    
#     # Roll Torque
#     axes[0, 1].plot(t, df['torque_x'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque X')
#     shade_angle_mode(axes[0, 1])
#     axes[0, 1].set_title('Roll: Control Output')
#     axes[0, 1].set_ylabel('Torque (N*m)')
#     axes[0, 1].grid(True)

#     # --- Row 1: Pitch ---
#     plot_dual_mode(
#         axes[1, 0], t, df['mode_id'], df['cmd_y'], 
#         df['act_q_rad_s'], df['act_pitch_rad'], "Pitch"
#     )
    
#     # Pitch Torque
#     axes[1, 1].plot(t, df['torque_y'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque Y')
#     shade_angle_mode(axes[1, 1])
#     axes[1, 1].set_title('Pitch: Control Output')
#     axes[1, 1].set_ylabel('Torque (N*m)')
#     axes[1, 1].grid(True)

#     # --- Row 2: Yaw ---
#     # User Request: "Yaw is always rate, same background color"
#     # So we treat it as pure Rate mode plot (Electric Teal / Hot Pink)
#     ax_yaw_dyn = axes[2, 0]
#     ax_yaw_dyn.plot(t, df['act_r_rad_s'], color=neon_sorbet["Electric Teal"], linewidth=1.5, label='Actual Rate (rad/s)')
#     ax_yaw_dyn.plot(t, df['cmd_z'], color=neon_sorbet["Hot Pink"], linestyle='--', linewidth=2, label='Cmd Rate (rad/s)')
#     ax_yaw_dyn.set_title('Yaw: Tracking (Always Rate)')
#     ax_yaw_dyn.set_ylabel('Magnitude')
#     ax_yaw_dyn.set_xlabel('Time (s)')
#     ax_yaw_dyn.legend(loc='upper left', fontsize='small')
#     ax_yaw_dyn.grid(True)
#     # Explicitly NOT calling shade_angle_mode(ax_yaw_dyn) per request

#     # Yaw Torque
#     ax_yaw_trq = axes[2, 1]
#     ax_yaw_trq.plot(t, df['torque_z'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque Z')
#     ax_yaw_trq.set_title('Yaw: Control Output')
#     ax_yaw_trq.set_ylabel('Torque (N*m)')
#     ax_yaw_trq.set_xlabel('Time (s)')
#     ax_yaw_trq.grid(True)

#     plt.tight_layout()
    
#     # Save figure
#     output_image = csv_path.replace('.csv', '.png')
#     plt.savefig(output_image, dpi=150)
#     print(f"Plot saved to: {output_image}")
    
#     # Show interactive plot
#     plt.show()

# if __name__ == "__main__":
#     # Default path relative to crate root
#     path = "tests/controller/rust_controller_results.csv"
#     plot_results(path)

import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# --- Color Scheme ---
neon_sorbet = {
    "Electric Teal": "#00DEDE",
    "Hot Pink": "#FF69B4",
    "Zesty Lime": "#89CA26",
    "Solar Flare Orange": "#FF4500",
    "Cyber Grape": "#9932CC",
}

def plot_results(csv_path):
    # Verify file existence
    if not os.path.exists(csv_path):
        print(f"Error: Could not find '{csv_path}'")
        sys.exit(1)

    # Load CSV data
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    # Setup plot aesthetics
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # Create 3x2 grid
    # Left Column: Tracking (Command vs Actual)
    # Right Column: Control Effort (Torque)
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig.suptitle(f'Controller Simulation: Rate -> Angle (Sine) -> Angle (Square)', fontsize=16)

    t = df['time_s']
    
    # Set x limits for all axes to 20s
    for ax in axes.flatten():
        ax.set_xlim(0, 20)

    # Helper to shade the background where Angle Mode is active
    # Fixed to handle multiple non-contiguous blocks of Angle mode
    def shade_angle_mode(ax):
        is_angle = df['mode_id'] == 1
        
        # Group consecutive rows with the same mode to identify contiguous blocks
        groups = (df['mode_id'] != df['mode_id'].shift()).cumsum()
        
        # Iterate only over the groups that are actually Angle mode
        for _, group in df[is_angle].groupby(groups):
            start_t = group['time_s'].iloc[0]
            end_t = group['time_s'].iloc[-1]
            ax.axvspan(start_t, end_t, color='gray', alpha=0.15, lw=0)

    # Helper to plot Rate/Angle split
    def plot_dual_mode(ax, time, mode_series, cmd_series, rate_act, angle_act, title_prefix):
        mask_rate = mode_series == 0
        mask_angle = mode_series == 1
        
        # Rate Section (Electric Teal / Hot Pink)
        if mask_rate.any():
            ax.plot(time[mask_rate], rate_act[mask_rate], color=neon_sorbet["Electric Teal"], linewidth=1.5, label='Actual Rate (rad/s)')
            ax.plot(time[mask_rate], cmd_series[mask_rate], color=neon_sorbet["Hot Pink"], linestyle='--', linewidth=2, label='Cmd Rate (rad/s)')
            
        # Angle Section (Zesty Lime / Solar Flare Orange)
        if mask_angle.any():
            ax.plot(time[mask_angle], angle_act[mask_angle], color=neon_sorbet["Zesty Lime"], linewidth=1.5, label='Actual Angle (rad)')
            ax.plot(time[mask_angle], cmd_series[mask_angle], color=neon_sorbet["Solar Flare Orange"], linestyle='--', linewidth=2, label='Cmd Angle (rad)')
            
        shade_angle_mode(ax)
        ax.set_title(f'{title_prefix}: Tracking')
        ax.legend(loc='upper left', fontsize='small')
        ax.grid(True)

    # --- Row 0: Roll ---
    plot_dual_mode(
        axes[0, 0], t, df['mode_id'], df['cmd_x'], 
        df['act_p_rad_s'], df['act_roll_rad'], "Roll"
    )
    
    # Roll Torque
    axes[0, 1].plot(t, df['torque_x'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque X')
    shade_angle_mode(axes[0, 1])
    axes[0, 1].set_title('Roll: Control Output')
    axes[0, 1].set_ylabel('Torque (N*m)')
    axes[0, 1].grid(True)

    # --- Row 1: Pitch ---
    plot_dual_mode(
        axes[1, 0], t, df['mode_id'], df['cmd_y'], 
        df['act_q_rad_s'], df['act_pitch_rad'], "Pitch"
    )
    
    # Pitch Torque
    axes[1, 1].plot(t, df['torque_y'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque Y')
    shade_angle_mode(axes[1, 1])
    axes[1, 1].set_title('Pitch: Control Output')
    axes[1, 1].set_ylabel('Torque (N*m)')
    axes[1, 1].grid(True)

    # --- Row 2: Yaw ---
    # Yaw is always rate, same background color
    ax_yaw_dyn = axes[2, 0]
    ax_yaw_dyn.plot(t, df['act_r_rad_s'], color=neon_sorbet["Electric Teal"], linewidth=1.5, label='Actual Rate (rad/s)')
    ax_yaw_dyn.plot(t, df['cmd_z'], color=neon_sorbet["Hot Pink"], linestyle='--', linewidth=2, label='Cmd Rate (rad/s)')
    ax_yaw_dyn.set_title('Yaw: Tracking (Always Rate)')
    ax_yaw_dyn.set_ylabel('Magnitude')
    ax_yaw_dyn.set_xlabel('Time (s)')
    ax_yaw_dyn.legend(loc='upper left', fontsize='small')
    ax_yaw_dyn.grid(True)

    # Yaw Torque
    ax_yaw_trq = axes[2, 1]
    ax_yaw_trq.plot(t, df['torque_z'], color=neon_sorbet["Cyber Grape"], alpha=0.8, label='Torque Z')
    ax_yaw_trq.set_title('Yaw: Control Output')
    ax_yaw_trq.set_ylabel('Torque (N*m)')
    ax_yaw_trq.set_xlabel('Time (s)')
    ax_yaw_trq.grid(True)

    plt.tight_layout()
    
    # Save figure
    output_image = csv_path.replace('.csv', '.png')
    plt.savefig(output_image, dpi=150)
    print(f"Plot saved to: {output_image}")
    
    # Show interactive plot
    plt.show()

if __name__ == "__main__":
    # Default path relative to crate root
    path = "tests/controller/rust_controller_results.csv"
    plot_results(path)