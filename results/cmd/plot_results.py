#!/usr/bin/env python3

import os
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
from typing import Dict, List, Tuple

# Use non-interactive backend
matplotlib.use('Agg')

def parse_test_file(filepath: str) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    Parse test output file and extract metrics.
    Returns: (astar_times, astar_vals, dlite_times, dlite_vals)
    """
    astar_times = []
    astar_vals = []
    dlite_times = []
    dlite_vals = []
    
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            # Skip header lines (first 3 lines)
            for line in lines[3:]:
                line = line.strip()
                if not line:
                    continue
                # Split by pipe character
                parts = [p.strip() for p in line.split('|')]
                if len(parts) >= 7:
                    try:
                        astar_times.append(float(parts[2]))
                        astar_vals.append(float(parts[3]))
                        dlite_times.append(float(parts[5]))
                        dlite_vals.append(float(parts[6]))
                    except (ValueError, IndexError):
                        continue
    except FileNotFoundError:
        pass
    
    return astar_times, astar_vals, dlite_times, dlite_vals

def calculate_average(values: List[float]) -> float:
    """Calculate average of a list of values."""
    if not values:
        return 0.0
    return sum(values) / len(values)

def extract_steps(filepath: str) -> int:
    """Extract objects_per_step from the file header."""
    try:
        with open(filepath, 'r') as f:
            for line in f:
                if "Detected from header" in line and "objects_per_step" in line:
                    parts = line.split("objects_per_step=")
                    if len(parts) > 1:
                        return int(parts[1].split()[0])
    except (FileNotFoundError, ValueError):
        pass
    return 0

def main():
    script_dir = Path(__file__).parent
    root_dir = script_dir.parent.parent
    results_dir = root_dir / "results"
    
    # Prepare data for plotting
    test_names = ["0_dst", "10_dst", "30_dst", "50_dst", "70_dst"]
    colors = ['gray', 'blue', 'green', 'orange', 'red']
    
    # Create figure for dynamic tests
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Collect all data first to find min/max for consistent scaling
    astar_all_times = []
    dlite_all_times = []
    
    for test_name in test_names:
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue
        
        for dynamic_file in test_dir.glob("dynamic_5000_*.txt"):
            astar_times, _, dlite_times, _ = parse_test_file(str(dynamic_file))
            if astar_times:
                astar_all_times.extend(astar_times)
            if dlite_times:
                dlite_all_times.extend(dlite_times)
    
    # Set fixed y-axis limit
    y_max = 20
    
    # Plot A* dynamic tests
    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue
        
        data_points = []
        
        for dynamic_file in test_dir.glob("dynamic_5000_*.txt"):
            steps = extract_steps(str(dynamic_file))
            astar_times, _, _, _ = parse_test_file(str(dynamic_file))
            
            if astar_times and steps >= 0:
                data_points.append((steps, calculate_average(astar_times)))
        
        if data_points:
            # Sort by steps (quantity of changes)
            data_points.sort(key=lambda x: x[0])
            steps_list = [x[0] for x in data_points]
            times_list = [x[1] for x in data_points]
            ax1.plot(steps_list, times_list, marker='o', label=test_name, color=color, linewidth=2)
    
    ax1.set_xlabel('Количество изменяющихся препятствий за ход', fontsize=11)
    ax1.set_ylabel('Среднее время (мс)', fontsize=11)
    ax1.set_title('Динамические тесты A*', fontsize=12, fontweight='bold')
    ax1.set_ylim(0, y_max)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot D*Lite dynamic tests
    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue
        
        data_points = []
        
        for dynamic_file in test_dir.glob("dynamic_5000_*.txt"):
            steps = extract_steps(str(dynamic_file))
            _, _, dlite_times, _ = parse_test_file(str(dynamic_file))
            
            if dlite_times and steps >= 0:
                data_points.append((steps, calculate_average(dlite_times)))
        
        if data_points:
            # Sort by steps (quantity of changes)
            data_points.sort(key=lambda x: x[0])
            steps_list = [x[0] for x in data_points]
            times_list = [x[1] for x in data_points]
            ax2.plot(steps_list, times_list, marker='s', label=test_name, color=color, linewidth=2)
    
    ax2.set_xlabel('Количество изменяющихся препятствий за ход', fontsize=11)
    ax2.set_ylabel('Среднее время (мс)', fontsize=11)
    ax2.set_title('Динамические тесты D* Lite', fontsize=12, fontweight='bold')
    ax2.set_ylim(0, y_max)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = results_dir / "dynamic_tests_comparison.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Graph saved to: {output_file}")
    
    # Create figure for static tests comparison
    fig2, ax3 = plt.subplots(figsize=(10, 6))
    
    static_data = {}
    for test_name, color in zip(test_names, colors):
        test_dir = results_dir / test_name
        if not test_dir.exists():
            continue
        
        static_file = test_dir / "static.txt"
        if static_file.exists():
            astar_times, astar_lens, dlite_times, dlite_lens = parse_test_file(str(static_file))
            
            static_data[test_name] = {
                'astar_time': calculate_average(astar_times),
                'astar_len': calculate_average(astar_lens),
                'dlite_time': calculate_average(dlite_times),
                'dlite_len': calculate_average(dlite_lens)
            }
    
    # Line chart for static tests (A* vs D*Lite)
    import numpy as np
    
    x = np.arange(len(static_data))
    
    astar_times = [static_data[name]['astar_time'] for name in static_data.keys()]
    dlite_times = [static_data[name]['dlite_time'] for name in static_data.keys()]
    
    ax3.plot(x, astar_times, marker='o', label='A*', color='skyblue', linewidth=2, markersize=8)
    ax3.plot(x, dlite_times, marker='s', label='D*Lite', color='salmon', linewidth=2, markersize=8)
    
    ax3.set_xlabel('Конфигурация теста', fontsize=11)
    ax3.set_ylabel('Среднее время (мс)', fontsize=11)
    ax3.set_title('Сравнение статических тестов', fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(static_data.keys())
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file2 = results_dir / "static_tests_comparison.png"
    plt.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"Graph saved to: {output_file2}")
    
    print("Graphs generated successfully!")

if __name__ == "__main__":
    main()
