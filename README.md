# RO07 - EKF SLAM Implementation

C++ implementation of Extended Kalman Filter Simultaneous Localization and Mapping (EKF-SLAM) for FPGA deployment.

## Overview

This project implements EKF-SLAM in C++, making it suitable for FPGA synthesis. The implementation is validated against a Python reference implementation.

## Features

- **Pure C++ implementation**
- **FPGA-ready** - Custom matrix operations suitable for hardware synthesis
- **High performance** - 140x faster than Python reference
- **Validated** - Testbench compares against Python reference data
- **Known data association** - Uses ground truth landmark IDs

## Project Structure

```
.
├── ekf_slam.h                      # EKF SLAM class header
├── ekf_slam.cpp                    # EKF SLAM implementation
├── ekf_slam_dataloader.h           # Data loading utilities header
├── ekf_slam_dataloader.cpp         # CSV/text file parser
├── testbench.cpp                   # Validation testbench
├── ekf_slam_initial_conditions.txt # Initial state and parameters
└── ekf_slam_data.csv               # Reference data from Python
```

## Building

### Prerequisites
- C++11 compatible compiler (g++, clang++)
- CMake 3.10 or higher (optional)
- Make

### Build with CMake
```bash
mkdir build
cd build
cmake ..
make
./ekf_slam_testbench
```

### Build without CMake
```bash
g++ -std=c++11 -O2 -o ekf_slam_testbench testbench.cpp ekf_slam.cpp ekf_slam_dataloader.cpp
./ekf_slam_testbench
```

## Running the Testbench

```bash
./ekf_slam_testbench
```

The testbench will:
1. Load initial conditions
2. Load reference iteration data from CSV
3. Run EKF-SLAM for all iterations
4. Compare C++ results against Python reference
5. Report validation results and performance metrics

## Output

```
========================================
Validation Results
========================================
Total iterations processed: 801
Maximum state error: 1.276715e+00
Maximum covariance error: 7.881247e-02

=== Performance Comparison ===
Average C++ execution time: 0.004 ms
Average Python execution time: 0.567 ms
Speedup: 140.359x
```

## Known Issues

- Numerical divergence starting at iteration 407 (under investigation)
- First 406 iterations match Python reference perfectly
- Divergence may be related to angle wrapping near -π

## Performance

- **Speed**: 140x faster than Python
- **Memory**: No dynamic allocations in main loop
- **Accuracy**: Perfect match for first 406 iterations