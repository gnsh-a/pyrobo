#!/usr/bin/env python3
"""
Script to create a truncated version of the tip trajectories comparison plot
showing only data up to t = 0.5 seconds.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

def load_tip_positions_csv(n_elements, h=5e-4):
    """Load tip positions from CSV file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_filename = f'tip_positions_refined_{n_elements}_h{h:.0e}.csv'
    csv_path = os.path.join(script_dir, csv_filename)
    
    if not os.path.exists(csv_path):
        print(f"Warning: CSV file not found: {csv_path}")
        return None
    
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    return {
        'time': data[:, 0],
        'tip_x': data[:, 1],
        'tip_y': data[:, 2],
        'tip_z': data[:, 3],
        'force_z': data[:, 4]
    }

def plot_trajectory_comparison_truncated(all_data, t_max=0.5, h=5e-4):
    """Create comparison plots for all element counts, truncated to t_max."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create figure with subplots for X, Y, Z positions
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))
    
    for idx, (n_elem, data) in enumerate(sorted(all_data.items())):
        if data is None:
            continue
        label = f'{n_elem} element{"s" if n_elem > 1 else ""}'
        color = colors[idx]
        
        # Filter data to only show up to t_max
        mask = data['time'] <= t_max
        time_filtered = data['time'][mask]
        tip_x_filtered = data['tip_x'][mask]
        tip_y_filtered = data['tip_y'][mask]
        tip_z_filtered = data['tip_z'][mask]
        
        # Plot X position
        axes[0].plot(time_filtered, tip_x_filtered, '-', color=color, 
                   label=label, linewidth=1.5)
        
        # Plot Y position
        axes[1].plot(time_filtered, tip_y_filtered, '-', color=color, 
                   label=label, linewidth=1.5)
        
        # Plot Z position
        axes[2].plot(time_filtered, tip_z_filtered, '-', color=color, 
                   label=label, linewidth=1.5)
    
    # Format X position subplot
    axes[0].set_title("Tip X Position Comparison", fontsize=14, fontweight='bold')
    axes[0].set_xlabel("Time (s)", fontsize=12)
    axes[0].set_ylabel("Global X Position (m)", fontsize=12)
    axes[0].set_xlim([0, t_max])
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', fontsize=10)
    
    # Format Y position subplot
    axes[1].set_title("Tip Y Position Comparison", fontsize=14, fontweight='bold')
    axes[1].set_xlabel("Time (s)", fontsize=12)
    axes[1].set_ylabel("Global Y Position (m)", fontsize=12)
    axes[1].set_xlim([0, t_max])
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=10)
    
    # Format Z position subplot
    axes[2].set_title("Tip Z Position Comparison", fontsize=14, fontweight='bold')
    axes[2].set_xlabel("Time (s)", fontsize=12)
    axes[2].set_ylabel("Global Z Position (m)", fontsize=12)
    axes[2].set_xlim([0, t_max])
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='best', fontsize=10)
    
    plt.tight_layout()
    output_path = os.path.join(script_dir, 'tip_trajectories_comparison_t0.5.png')
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nTruncated comparison plot saved: {output_path}")
    plt.close(fig)

def main():
    """Main function to generate truncated plot."""
    # Element counts to load
    n_elements_list = [1, 2, 3, 4, 8, 16]
    
    all_data = {}
    
    print("Loading CSV data...")
    for n_elements in n_elements_list:
        data = load_tip_positions_csv(n_elements)
        if data is not None:
            all_data[n_elements] = data
            print(f"  Loaded data for {n_elements} element(s)")
        else:
            print(f"  Warning: Could not load data for {n_elements} element(s)")
    
    if not all_data:
        print("Error: No data loaded. Please run simulations first.")
        return
    
    print("\nGenerating truncated comparison plot (t <= 0.5 s)...")
    plot_trajectory_comparison_truncated(all_data, t_max=0.5)
    print("\nDone!")

if __name__ == "__main__":
    main()

