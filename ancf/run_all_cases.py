#!/usr/bin/env python3
"""
Wrapper script to run main-mbd-deformable.py for multiple element counts.
Generates comparison plots for trajectories and computation time.
"""

import subprocess
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import glob

def run_simulation(n_elements):
    """Run main-mbd-deformable.py for a given number of elements."""
    print(f"\n{'='*60}")
    print(f"Running simulation with {n_elements} element(s)")
    print(f"{'='*60}")
    
    script_path = os.path.join(os.path.dirname(__file__), 'main-mbd-deformable.py')
    
    start_time = time.perf_counter()
    
    try:
        result = subprocess.run(
            ['python3', script_path, str(n_elements)],
            check=True,
            capture_output=False,  # Show output in real-time
            text=True
        )
        elapsed_time = time.perf_counter() - start_time
        print(f"\nCompleted {n_elements} element(s) in {elapsed_time:.2f} seconds")
        return elapsed_time, True
    except subprocess.CalledProcessError as e:
        elapsed_time = time.perf_counter() - start_time
        print(f"\nError running {n_elements} element(s): {e}")
        return elapsed_time, False

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

def plot_trajectory_comparison(all_data, timing_data, h=5e-4):
    """Create comparison plots for all element counts."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create figure with subplots for X, Y, Z positions
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))
    
    for idx, (n_elem, data) in enumerate(sorted(all_data.items())):
        if data is None:
            continue
        label = f'{n_elem} element{"s" if n_elem > 1 else ""}'
        color = colors[idx]
        
        # Plot X position
        axes[0].plot(data['time'], data['tip_x'], '-', color=color, 
                   label=label, linewidth=1.5)
        
        # Plot Y position
        axes[1].plot(data['time'], data['tip_y'], '-', color=color, 
                   label=label, linewidth=1.5)
        
        # Plot Z position
        axes[2].plot(data['time'], data['tip_z'], '-', color=color, 
                   label=label, linewidth=1.5)
    
    # Format X position subplot
    axes[0].set_title("Tip X Position Comparison", fontsize=14, fontweight='bold')
    axes[0].set_xlabel("Time (s)", fontsize=12)
    axes[0].set_ylabel("Global X Position (m)", fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', fontsize=10)
    
    # Format Y position subplot
    axes[1].set_title("Tip Y Position Comparison", fontsize=14, fontweight='bold')
    axes[1].set_xlabel("Time (s)", fontsize=12)
    axes[1].set_ylabel("Global Y Position (m)", fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=10)
    
    # Format Z position subplot
    axes[2].set_title("Tip Z Position Comparison", fontsize=14, fontweight='bold')
    axes[2].set_xlabel("Time (s)", fontsize=12)
    axes[2].set_ylabel("Global Z Position (m)", fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='best', fontsize=10)
    
    plt.tight_layout()
    output_path = os.path.join(script_dir, 'tip_trajectories_comparison.png')
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nComparison plot saved: {output_path}")
    plt.close(fig)
    
    # Create timing plot
    fig_time, ax_time = plt.subplots(1, 1, figsize=(10, 6))
    
    n_elements_list = sorted([n for n, success in timing_data.items() if success])
    times_list = [timing_data[n] for n in n_elements_list]
    
    ax_time.plot(n_elements_list, times_list, 'o-', linewidth=2, markersize=8, 
                color='steelblue', markerfacecolor='lightblue', markeredgewidth=2)
    ax_time.set_xlabel("Number of Elements", fontsize=12, fontweight='bold')
    ax_time.set_ylabel("Computation Time (seconds)", fontsize=12, fontweight='bold')
    ax_time.set_title("Computation Time vs Number of Elements", fontsize=14, fontweight='bold')
    ax_time.grid(True, alpha=0.3)
    ax_time.set_xscale('log', base=2)
    ax_time.set_yscale('log')
    
    # Add value labels on points
    for n, t in zip(n_elements_list, times_list):
        ax_time.annotate(f'{t:.1f}s', (n, t), 
                        textcoords="offset points", xytext=(0,10), 
                        ha='center', fontsize=9)
    
    plt.tight_layout()
    output_path_time = os.path.join(script_dir, 'computation_time_vs_elements.png')
    fig_time.savefig(output_path_time, dpi=300, bbox_inches='tight')
    print(f"Timing plot saved: {output_path_time}")
    plt.close(fig_time)

def main():
    """Main function to run all simulations and generate comparison plots."""
    # Element counts to test
    n_elements_list = [1, 2, 3, 4, 8, 16]
    
    timing_data = {}
    all_data = {}
    
    print("="*60)
    print("BATCH SIMULATION RUNNER")
    print("="*60)
    print(f"Will run simulations for: {n_elements_list}")
    print(f"Each simulation runs for 10 seconds")
    print("="*60)
    
    # Run all simulations
    for n_elements in n_elements_list:
        elapsed_time, success = run_simulation(n_elements)
        timing_data[n_elements] = elapsed_time if success else None
        
        if success:
            # Load the CSV data
            data = load_tip_positions_csv(n_elements)
            all_data[n_elements] = data
        else:
            all_data[n_elements] = None
    
    # Generate comparison plots
    print("\n" + "="*60)
    print("GENERATING COMPARISON PLOTS")
    print("="*60)
    
    plot_trajectory_comparison(all_data, timing_data)
    
    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"{'Elements':<10} {'Time (s)':<15} {'Status':<10}")
    print("-" * 60)
    for n_elem in n_elements_list:
        if n_elem in timing_data and timing_data[n_elem] is not None:
            status = "Success"
            time_str = f"{timing_data[n_elem]:.2f}"
        else:
            status = "Failed"
            time_str = "N/A"
        print(f"{n_elem:<10} {time_str:<15} {status:<10}")
    print("="*60)
    print("\nAll simulations complete!")

if __name__ == "__main__":
    main()

