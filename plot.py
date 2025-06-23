import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load the CSV
df = pd.read_csv("joint_velocities.csv")

# Get joint names from the header
joint_names = df.columns[2:]

# Create a color palette for clarity
palette = sns.color_palette("tab10", len(joint_names))

for joint in joint_names:
    plt.figure(figsize=(10, 6))
    for vsf in sorted(df['velocity_scaling_factor'].unique()):
        sub_df = df[df['velocity_scaling_factor'] == vsf]
        plt.plot(sub_df['time_from_start'], sub_df[joint],
                 label=f"VSF: {vsf}", linewidth=2)
    
    plt.title(f"Joint Velocity: {joint}")
    plt.xlabel("Time from start (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
