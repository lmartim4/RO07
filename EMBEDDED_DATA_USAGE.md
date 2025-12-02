# Using Embedded Data (No SD Card Required)

This guide explains how to use the embedded data version of the testbench for the Zed Board, which doesn't require an SD card or CSV file.

## Overview

The embedded data system includes:
- **csv_to_header.py**: Python script to convert CSV files to C header files
- **ekf_slam_data.h**: Generated header file with embedded data arrays
- **ekf_slam_dataloader_embedded.cpp/h**: Data loader that reads from the header instead of CSV
- **testbench_embedded.cpp**: Testbench that uses the embedded data

## Step 1: Generate the Header File

First, convert your CSV file to a header file:

```bash
python csv_to_header.py ekf_slam_data.csv
```

This creates `ekf_slam_data.h` with all the CSV data embedded as C arrays.

### Custom Output Name (Optional)

```bash
python csv_to_header.py ekf_slam_data.csv my_custom_data.h
```

## Step 2: Build the Embedded Testbench

The CMakeLists.txt has been updated to include the embedded testbench target:

```bash
cd build
cmake ..
make ekf_slam_testbench_embedded
```

Or build all targets:

```bash
make
```

## Step 3: Run the Embedded Testbench

The embedded testbench still needs the initial conditions file but NOT the CSV file:

```bash
./ekf_slam_testbench_embedded
```

Required file:
- `ekf_slam_initial_conditions.txt` ✓

NOT required:
- `ekf_slam_data.csv` ✗ (data is embedded in the executable)

## Comparison: Original vs Embedded

### Original Testbench (testbench.cpp)
```cpp
#include "ekf_slam_dataloader.h"

EKFSLAMDataLoader loader;
loader.loadIterationData("ekf_slam_data.csv");  // Reads from CSV file
```

**Requirements:**
- ekf_slam_initial_conditions.txt
- ekf_slam_data.csv

### Embedded Testbench (testbench_embedded.cpp)
```cpp
#include "ekf_slam_dataloader_embedded.h"
#include "ekf_slam_data.h"  // Contains embedded data

EKFSLAMDataLoaderEmbedded loader;
loader.loadIterationDataFromHeader();  // Reads from embedded arrays
```

**Requirements:**
- ekf_slam_initial_conditions.txt only!

## Benefits for Zed Board

1. **No SD Card Needed**: All iteration data is compiled into the executable
2. **Faster Access**: No file I/O overhead during runtime
3. **Single Binary**: Everything except initial conditions is embedded
4. **Smaller Footprint**: One large executable instead of executable + large CSV

## File Size Considerations

The generated header file will be large (approximately the same size as the CSV file or larger). This is expected because:
- CSV data: ~X MB
- Header file: ~Y MB (includes formatting)
- Compiled binary: ~Z MB (includes compressed data)

The executable will be larger, but this is fine for the Zed Board's memory capacity.

## Header File Structure

The generated `ekf_slam_data.h` contains:

```c
#define NUM_DATA_ROWS 406          // Number of iterations
#define MAX_ROW_LENGTH 52          // Maximum columns in any row

static const int row_lengths[NUM_DATA_ROWS] = { ... };
static const float csv_data[NUM_DATA_ROWS][MAX_ROW_LENGTH] = { ... };

// Helper function
inline float get_csv_value(int row, int col) { ... }
```

## Advanced: Customizing the Python Script

The `csv_to_header.py` script can be modified to:
- Change data types (float/double)
- Add compression
- Generate multiple header files for large datasets
- Include additional metadata

## Troubleshooting

### "File too large" compilation errors
- Your CSV might be too large to embed
- Consider reducing the dataset or splitting into multiple headers

### Missing `ekf_slam_data.h`
- Run `python csv_to_header.py ekf_slam_data.csv` first
- Make sure the header is in the project root directory

### Different results between testbenches
- Both should produce identical results
- If not, check that the header was generated from the latest CSV

## For HLS/Zed Board Deployment

When deploying to the Zed Board:
1. Generate the header file on your development machine
2. Include `ekf_slam_data.h` in your Vivado HLS or SDK project
3. Use `testbench_embedded.cpp` as your testbench
4. Only copy `ekf_slam_initial_conditions.txt` to the board (if needed)
5. The executable will have all iteration data built-in
