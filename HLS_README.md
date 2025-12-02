# EKF SLAM HLS Acceleration

This directory contains an HLS-optimized version of the EKF SLAM implementation for FPGA acceleration.

## Architecture Overview

The implementation separates software control from hardware acceleration:

```
Software (CPU)                  Hardware (FPGA)
+------------------+            +------------------+
| EKFSLAM_HLS      |  <------>  | ekf_slam_update  |
| (Wrapper)        |            | _hls (Kernel)    |
+------------------+            +------------------+
| - Data format    |            | - Fixed arrays   |
|   conversion     |            | - Matrix ops     |
| - Memory mgmt    |            | - Kalman filter  |
+------------------+            +------------------+
```

## Files

### HLS Kernel Files (Synthesizable)
- **ekf_slam_hls.h** - HLS-compatible data structures and kernel interface
- **ekf_slam_hls.cpp** - Synthesizable kernel implementation with HLS pragmas

### Software Wrapper Files
- **ekf_slam_hls_wrapper.h** - Wrapper class header
- **ekf_slam_hls_wrapper.cpp** - Bridge between software EKFSLAM and HLS kernel

### Original Software Implementation
- **ekf_slam.h / ekf_slam.cpp** - Original implementation (unchanged)
- **ekf_slam_dataloader.h / ekf_slam_dataloader.cpp** - Data loading (unchanged)

## Key Design Decisions

### 1. Fixed-Size Arrays
- Maximum landmarks: `MAX_LANDMARKS = 50`
- Maximum state size: `MAX_STATE_SIZE = 103` (3 + 2*50)
- Maximum observations per update: `MAX_OBSERVATIONS = 10`

These can be adjusted in [ekf_slam_hls.h:7-10](ekf_slam_hls.h#L7-L10) based on your requirements and FPGA resources.

### 2. Data Structures

**EKFStateHLS**: Fixed-size state representation
```cpp
struct EKFStateHLS {
    double xEst[MAX_STATE_SIZE];              // State vector
    double PEst[MAX_STATE_SIZE * MAX_STATE_SIZE]; // Covariance (flattened)
    int current_state_size;                    // Actual size in use
    int num_landmarks;                         // Number of landmarks
    int landmark_ids[MAX_LANDMARKS];           // Landmark ID tracking
};
```

**EKFParamsHLS**: Algorithm parameters
```cpp
struct EKFParamsHLS {
    double DT;                    // Time step
    double MAX_RANGE;             // Maximum sensor range
    double M_DIST_TH;             // Mahalanobis distance threshold
    double Q[4];                  // Process noise (2x2)
    double Py[4];                 // Observation noise (2x2)
    bool KNOWN_DATA_ASSOCIATION;  // Use known data association
};
```

### 3. HLS Pragmas

**Interface Pragmas** ([ekf_slam_hls.cpp:527-533](ekf_slam_hls.cpp#L527-L533)):
```cpp
#pragma HLS INTERFACE m_axi port=state bundle=gmem0
#pragma HLS INTERFACE m_axi port=params bundle=gmem1
#pragma HLS INTERFACE m_axi port=observations bundle=gmem2
#pragma HLS INTERFACE s_axilite port=return bundle=control
```

**Loop Optimization**:
- `#pragma HLS loop_tripcount` - Provides loop bounds for synthesis
- `#pragma HLS PIPELINE II=1` - Enables pipelining with initiation interval of 1
- `#pragma HLS UNROLL` - Unrolls small loops for parallelism

**Function Inlining**:
- `#pragma HLS INLINE` - Inlines small utility functions
- `#pragma HLS INLINE off` - Prevents inlining of large functions

## HLS Compatibility Features

### ✅ Resolved Issues from Original Code

1. **Dynamic Memory** → Fixed-size arrays
2. **std::vector** → C-style arrays with size tracking
3. **std::cout** → Removed or guarded with `#ifndef __SYNTHESIS__`
4. **Variable loop bounds** → Loop tripcount pragmas
5. **STL algorithms** → Manual loops
6. **Runtime-sized matrices** → Fixed maximum sizes

### ⚠️ Known Limitations

1. **Maximum Landmarks**: Hard limit of 50 landmarks (configurable)
2. **State Size Growth**: Falls back to software if limit exceeded
3. **Data Association**: Simplified - only known association implemented
4. **Floating-Point**: Uses double precision (consider fixed-point for optimization)

## Usage

### Basic Usage

```cpp
#include "ekf_slam_hls_wrapper.h"

int main() {
    // Create HLS-accelerated SLAM instance
    EKFSLAM_HLS slam;

    // Load initial conditions
    EKFSLAMDataLoader loader;
    loader.loadInitialConditions("ekf_slam_initial_conditions.txt");

    // Initialize HLS kernel
    slam.initializeHLS(loader.getInitialConditions());

    // Run SLAM loop
    double u[2] = {1.0, 0.1};  // Control input
    std::vector<Observation> observations = {...};

    slam.update(u, observations);  // Automatically uses HLS

    // Get results
    const Vector& state = slam.getStateEstimate();
    const Matrix& covariance = slam.getCovarianceEstimate();

    return 0;
}
```

### Software Fallback

```cpp
// Disable HLS acceleration (for debugging/comparison)
slam.setHLSEnabled(false);
slam.update(u, observations);  // Uses software implementation

// Re-enable HLS
slam.setHLSEnabled(true);
```

The wrapper automatically falls back to software if:
- HLS is not initialized
- HLS is disabled
- State size exceeds `MAX_STATE_SIZE`
- Number of landmarks exceeds `MAX_LANDMARKS`

## Building

### For Software Simulation

```bash
# Using CMake
mkdir build && cd build
cmake ..
make

# Manual compilation
g++ -std=c++11 -O3 \
    ekf_slam.cpp \
    ekf_slam_dataloader.cpp \
    ekf_slam_hls.cpp \
    ekf_slam_hls_wrapper.cpp \
    testbench.cpp \
    -o ekf_slam_test
```

### For HLS Synthesis

Using Vitis HLS:

```tcl
# create_hls_project.tcl
open_project ekf_slam_hls_project
set_top ekf_slam_update_hls

add_files ekf_slam_hls.cpp
add_files ekf_slam_hls.h
add_files -tb testbench_hls.cpp

open_solution "solution1"
set_part {xcvu9p-flga2104-2-i}  # Adjust for your FPGA
create_clock -period 10 -name default

# Synthesis
csynth_design

# Co-simulation (optional)
#cosim_design

# Export (optional)
#export_design -format ip_catalog
```

Run synthesis:
```bash
vitis_hls -f create_hls_project.tcl
```

## Performance Considerations

### Resource Usage (Estimated)
- **BRAM**: ~100-200 blocks (for state/covariance storage)
- **DSP**: ~200-400 (for matrix operations)
- **LUTs**: ~50K-100K
- **FFs**: ~50K-100K

### Optimization Opportunities

1. **Fixed-Point Arithmetic**: Convert from `double` to `ap_fixed` for better performance
2. **Array Partitioning**: Add `#pragma HLS ARRAY_PARTITION` for matrix operations
3. **Dataflow**: Use `#pragma HLS DATAFLOW` for prediction/update pipeline
4. **Loop Unrolling**: Unroll more loops for parallelism (at cost of resources)

Example optimizations to add:

```cpp
// In ekf_slam_hls.cpp
void mat_mul(...) {
    #pragma HLS ARRAY_PARTITION variable=A cyclic factor=4 dim=2
    #pragma HLS ARRAY_PARTITION variable=B cyclic factor=4 dim=1
    // ... existing code
}
```

## Testing

### Unit Testing
Test individual matrix operations:
```cpp
// Test matrix multiplication
double A[4] = {1, 2, 3, 4};
double B[4] = {5, 6, 7, 8};
double C[4];
hls_utils::mat_mul(A, B, C, 2, 2, 2);
```

### Integration Testing
Compare HLS vs software results:
```cpp
EKFSLAM slam_sw;      // Software version
EKFSLAM_HLS slam_hw;  // Hardware version

// Run same inputs
slam_sw.update(u, observations);
slam_hw.update(u, observations);

// Compare results
Vector state_sw = slam_sw.getStateEstimate();
Vector state_hw = slam_hw.getStateEstimate();
// Assert they match within tolerance
```

## Troubleshooting

### Synthesis Errors

**"Cannot determine loop trip count"**
- Add `#pragma HLS loop_tripcount` to all loops
- Ensure loop bounds are compile-time constants or bounded

**"Unsupported operation"**
- Check for STL containers (std::vector, std::map, etc.)
- Check for dynamic memory allocation (new, malloc)
- Check for file I/O operations

**"Interface synthesis failed"**
- Verify all pointers have interface pragmas
- Check that arrays have depth specified
- Ensure AXI bundle names are unique

### Runtime Issues

**Incorrect results**
- Verify data format conversion in wrapper
- Check matrix flattening/unflattening (row-major order)
- Compare with software implementation step-by-step

**Performance lower than expected**
- Check for unresolved loop dependencies
- Add more aggressive pipelining
- Consider array partitioning for parallel access

## Next Steps

1. **Synthesis**: Run Vitis HLS to generate RTL
2. **Verification**: Use C/RTL co-simulation to verify correctness
3. **Integration**: Package as IP and integrate into Vivado design
4. **Optimization**: Profile and optimize based on resource/performance reports
5. **Fixed-Point**: Convert to `ap_fixed` for better FPGA efficiency

## References

- Original EKF SLAM implementation: [ekf_slam.cpp](ekf_slam.cpp)
- Vitis HLS User Guide: UG1399
- HLS Pragmas Reference: UG1253
