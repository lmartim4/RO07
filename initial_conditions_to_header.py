#!/usr/bin/env python3
"""
Initial Conditions to C Header Converter
Converts the ekf_slam_initial_conditions.txt file into a C/C++ header file with embedded data arrays.
"""

import sys
import os
from typing import List, Tuple

def parse_initial_conditions(filename: str) -> dict:
    """Parse the initial conditions file and extract all data."""
    data = {
        'state_size': 0,
        'lm_size': 0,
        'dt': 0.0,
        'max_range': 0.0,
        'm_dist_th': 0.0,
        'landmarks': [],
        'x_est': [],
        'p_est': [],
        'q_matrix': [],
        'py_matrix': []
    }

    current_section = None

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()

            # Skip empty lines
            if not line:
                continue

            # Check for section headers (comments)
            if line.startswith('#'):
                section_lower = line.lower()
                if 'landmarks' in section_lower:
                    current_section = 'landmarks'
                elif 'initial xest' in section_lower:
                    current_section = 'x_est'
                elif 'initial pest' in section_lower:
                    current_section = 'p_est'
                elif 'q matrix' in section_lower:
                    current_section = 'q_matrix'
                elif 'py matrix' in section_lower:
                    current_section = 'py_matrix'
                continue

            # Parse parameter definitions
            if '=' in line and current_section is None:
                key, value = line.split('=')
                key = key.strip()
                value = value.strip()

                if key == 'STATE_SIZE':
                    data['state_size'] = int(value)
                elif key == 'LM_SIZE':
                    data['lm_size'] = int(value)
                elif key == 'DT':
                    data['dt'] = float(value)
                elif key == 'MAX_RANGE':
                    data['max_range'] = float(value)
                elif key == 'M_DIST_TH':
                    data['m_dist_th'] = float(value)
                continue

            # Parse data rows for current section
            if current_section:
                values = [float(x) for x in line.split()]

                if current_section == 'landmarks':
                    data['landmarks'].append(values)
                elif current_section == 'x_est':
                    data['x_est'].append(values[0])
                elif current_section == 'p_est':
                    data['p_est'].append(values)
                elif current_section == 'q_matrix':
                    data['q_matrix'].append(values)
                elif current_section == 'py_matrix':
                    data['py_matrix'].append(values)

    return data

def format_float_value(value: float) -> str:
    """
    Format a float value for C code, ensuring proper decimal point notation.

    Args:
        value: Float value to format

    Returns:
        Formatted string with 'f' suffix and decimal point
    """
    formatted = f"{value:.10g}"
    # Add .0 if the number looks like an integer (no decimal point or exponent)
    if '.' not in formatted and 'e' not in formatted.lower():
        formatted += '.0'
    return formatted + 'f'

def generate_header(input_filename: str, output_filename: str = None):
    """
    Convert initial conditions file to C header file.

    Args:
        input_filename: Input initial conditions file path
        output_filename: Output header file path (defaults to input with .h extension)
    """
    if output_filename is None:
        base_name = os.path.splitext(input_filename)[0]
        output_filename = base_name + ".h"

    print(f"Reading initial conditions file: {input_filename}")
    data = parse_initial_conditions(input_filename)

    print(f"Parsed data:")
    print(f"  STATE_SIZE: {data['state_size']}")
    print(f"  LM_SIZE: {data['lm_size']}")
    print(f"  DT: {data['dt']}")
    print(f"  MAX_RANGE: {data['max_range']}")
    print(f"  M_DIST_TH: {data['m_dist_th']}")
    print(f"  Landmarks: {len(data['landmarks'])} rows")
    print(f"  xEst: {len(data['x_est'])} values")
    print(f"  PEst: {len(data['p_est'])} rows")
    print(f"  Q matrix: {len(data['q_matrix'])} rows")
    print(f"  Py matrix: {len(data['py_matrix'])} rows")

    # Generate header file
    print(f"\nGenerating header file: {output_filename}")

    guard_name = os.path.basename(output_filename).replace('.', '_').replace('-', '_').upper()

    with open(output_filename, 'w') as hfile:
        # Write header guard and includes
        hfile.write(f"#ifndef {guard_name}\n")
        hfile.write(f"#define {guard_name}\n\n")
        hfile.write("// Auto-generated header file from initial conditions\n")
        hfile.write(f"// Source: {os.path.basename(input_filename)}\n\n")

        # Write parameter constants
        # Use PARAM_ prefix to avoid naming conflicts with struct members
        hfile.write("// EKF SLAM Parameters\n")
        hfile.write(f"#define PARAM_STATE_SIZE {data['state_size']}\n")
        hfile.write(f"#define PARAM_LM_SIZE {data['lm_size']}\n")
        hfile.write(f"#define PARAM_DT {format_float_value(data['dt'])}\n")
        hfile.write(f"#define PARAM_MAX_RANGE {format_float_value(data['max_range'])}\n")
        hfile.write(f"#define PARAM_M_DIST_TH {format_float_value(data['m_dist_th'])}\n")
        hfile.write(f"#define PARAM_NUM_LANDMARKS {len(data['landmarks'])}\n\n")

        # Write landmarks array
        hfile.write("// Landmark positions [x, y]\n")
        hfile.write(f"static const float landmarks[PARAM_NUM_LANDMARKS][PARAM_LM_SIZE] = {{\n")
        for i, lm in enumerate(data['landmarks']):
            hfile.write("    {")
            hfile.write(", ".join([format_float_value(v) for v in lm]))
            hfile.write("}")
            if i < len(data['landmarks']) - 1:
                hfile.write(",")
            hfile.write("\n")
        hfile.write("};\n\n")

        # Write initial state estimate
        hfile.write("// Initial state estimate (xEst)\n")
        hfile.write(f"static const float initial_xEst[PARAM_STATE_SIZE] = {{\n")
        hfile.write("    ")
        hfile.write(", ".join([format_float_value(v) for v in data['x_est']]))
        hfile.write("\n};\n\n")

        # Write initial covariance matrix
        hfile.write("// Initial state covariance matrix (PEst)\n")
        hfile.write(f"static const float initial_PEst[PARAM_STATE_SIZE][PARAM_STATE_SIZE] = {{\n")
        for i, row in enumerate(data['p_est']):
            hfile.write("    {")
            hfile.write(", ".join([format_float_value(v) for v in row]))
            hfile.write("}")
            if i < len(data['p_est']) - 1:
                hfile.write(",")
            hfile.write("\n")
        hfile.write("};\n\n")

        # Write Q matrix (process noise covariance)
        q_size = len(data['q_matrix'])
        hfile.write("// Process noise covariance matrix (Q)\n")
        hfile.write(f"#define PARAM_Q_SIZE {q_size}\n")
        hfile.write(f"static const float Q_matrix[PARAM_Q_SIZE][PARAM_Q_SIZE] = {{\n")
        for i, row in enumerate(data['q_matrix']):
            hfile.write("    {")
            hfile.write(", ".join([format_float_value(v) for v in row]))
            hfile.write("}")
            if i < len(data['q_matrix']) - 1:
                hfile.write(",")
            hfile.write("\n")
        hfile.write("};\n\n")

        # Write Py matrix (observation noise covariance)
        py_size = len(data['py_matrix'])
        hfile.write("// Observation noise covariance matrix (Py)\n")
        hfile.write(f"#define PARAM_PY_SIZE {py_size}\n")
        hfile.write(f"static const float Py_matrix[PARAM_PY_SIZE][PARAM_PY_SIZE] = {{\n")
        for i, row in enumerate(data['py_matrix']):
            hfile.write("    {")
            hfile.write(", ".join([format_float_value(v) for v in row]))
            hfile.write("}")
            if i < len(data['py_matrix']) - 1:
                hfile.write(",")
            hfile.write("\n")
        hfile.write("};\n\n")

        # Close header guard
        hfile.write(f"#endif // {guard_name}\n")

    print(f"Successfully generated {output_filename}")

    # Print file size info
    file_size = os.path.getsize(output_filename)
    print(f"Output file size: {file_size:,} bytes ({file_size/1024:.2f} KB)")

def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python initial_conditions_to_header.py <input_file> [output_header_file]")
        print("\nExample:")
        print("  python initial_conditions_to_header.py ekf_slam_initial_conditions.txt")
        print("  python initial_conditions_to_header.py ekf_slam_initial_conditions.txt initial_conditions.h")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(input_file):
        print(f"Error: File '{input_file}' not found!")
        sys.exit(1)

    try:
        generate_header(input_file, output_file)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
