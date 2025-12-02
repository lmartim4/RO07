#!/usr/bin/env python3
"""
CSV to C Header Converter
Converts a CSV file into a C/C++ header file with embedded data arrays.
"""

import csv
import sys
import os
from typing import List

def parse_csv_line(row: List[str]) -> List[float]:
    """Parse a CSV row and convert all values to floats."""
    values = []
    for item in row:
        try:
            values.append(float(item))
        except ValueError:
            # Skip non-numeric values (like comments)
            continue
    return values

def generate_header(csv_filename: str, output_filename: str = None):
    """
    Convert CSV file to C header file.

    Args:
        csv_filename: Input CSV file path
        output_filename: Output header file path (defaults to csv_filename with .h extension)
    """
    if output_filename is None:
        base_name = os.path.splitext(csv_filename)[0]
        output_filename = base_name + ".h"

    # Read and parse CSV
    all_rows = []
    row_lengths = []
    max_row_length = 0

    print(f"Reading CSV file: {csv_filename}")

    with open(csv_filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for line_num, row in enumerate(reader, 1):
            # Skip comment lines
            if len(row) > 0 and row[0].strip().startswith('#'):
                print(f"Skipping comment line {line_num}")
                continue

            # Parse numeric values
            values = parse_csv_line(row)
            if values:
                all_rows.append(values)
                row_lengths.append(len(values))
                max_row_length = max(max_row_length, len(values))

                if len(all_rows) % 50 == 0:
                    print(f"Processed {len(all_rows)} rows...")

    print(f"Total rows processed: {len(all_rows)}")
    print(f"Maximum row length: {max_row_length}")

    # Generate header file
    print(f"Generating header file: {output_filename}")

    guard_name = os.path.basename(output_filename).replace('.', '_').replace('-', '_').upper()

    with open(output_filename, 'w') as hfile:
        # Write header guard and includes
        hfile.write(f"#ifndef {guard_name}\n")
        hfile.write(f"#define {guard_name}\n\n")
        hfile.write("// Auto-generated header file from CSV data\n")
        hfile.write(f"// Source: {os.path.basename(csv_filename)}\n\n")

        # Write constants
        hfile.write(f"#define NUM_DATA_ROWS {len(all_rows)}\n")
        hfile.write(f"#define MAX_ROW_LENGTH {max_row_length}\n\n")

        # Write row lengths array
        hfile.write("// Array containing the length of each data row\n")
        hfile.write(f"static const int row_lengths[NUM_DATA_ROWS] = {{\n")
        for i, length in enumerate(row_lengths):
            if i % 10 == 0:
                hfile.write("    ")
            hfile.write(f"{length}")
            if i < len(row_lengths) - 1:
                hfile.write(", ")
            if (i + 1) % 10 == 0 or i == len(row_lengths) - 1:
                hfile.write("\n")
        hfile.write("};\n\n")

        # Write data as 2D array (with padding for variable-length rows)
        hfile.write("// 2D array containing all CSV data\n")
        hfile.write("// Each row is padded with zeros to MAX_ROW_LENGTH\n")
        hfile.write(f"static const float csv_data[NUM_DATA_ROWS][MAX_ROW_LENGTH] = {{\n")

        for row_idx, row_data in enumerate(all_rows):
            hfile.write("    {")

            # Write the actual data
            for col_idx, value in enumerate(row_data):
                hfile.write(f"{value:.10g}")
                if col_idx < len(row_data) - 1:
                    hfile.write(", ")

            # Pad with zeros if needed
            if len(row_data) < max_row_length:
                if len(row_data) > 0:
                    hfile.write(", ")
                zeros_needed = max_row_length - len(row_data)
                hfile.write(", ".join(["0.0"] * zeros_needed))

            hfile.write("}")
            if row_idx < len(all_rows) - 1:
                hfile.write(",")
            hfile.write("\n")

            if (row_idx + 1) % 10 == 0:
                print(f"Written {row_idx + 1}/{len(all_rows)} rows...")

        hfile.write("};\n\n")

        # Add helper function to get data
        hfile.write("// Helper function to get a specific value from the data\n")
        hfile.write("// Returns 0.0 if indices are out of bounds\n")
        hfile.write("inline float get_csv_value(int row, int col) {\n")
        hfile.write("    if (row < 0 || row >= NUM_DATA_ROWS || col < 0 || col >= row_lengths[row]) {\n")
        hfile.write("        return 0.0f;\n")
        hfile.write("    }\n")
        hfile.write("    return csv_data[row][col];\n")
        hfile.write("}\n\n")

        # Close header guard
        hfile.write(f"#endif // {guard_name}\n")

    print(f"Successfully generated {output_filename}")

    # Print file size info
    file_size = os.path.getsize(output_filename)
    print(f"Output file size: {file_size:,} bytes ({file_size/1024:.2f} KB)")

def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python csv_to_header.py <input_csv_file> [output_header_file]")
        print("\nExample:")
        print("  python csv_to_header.py ekf_slam_data.csv")
        print("  python csv_to_header.py ekf_slam_data.csv ekf_data.h")
        sys.exit(1)

    csv_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)

    try:
        generate_header(csv_file, output_file)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
