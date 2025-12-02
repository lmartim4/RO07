# EKF SLAM HLS Implementation Summary

## Overview

Your EKF SLAM implementation has been successfully prepared for FPGA acceleration using High-Level Synthesis (HLS). The solution uses **Option B: Separate HLS Kernel** approach, keeping your original code intact while creating HLS-optimized versions.

## Files Created

### Core HLS Files (Synthesizable for FPGA)
| File | Description | Lines | Purpose |
|------|-------------|-------|---------|
| `ekf_slam_hls.h` | HLS header with data structures | ~150 | Fixed-size data structures, kernel interface |
| `ekf_slam_hls.cpp` | HLS kernel implementation | ~700 | Synthesizable EKF SLAM algorithm |

### Software Integration Files
| File | Description | Lines | Purpose |
|------|-------------|-------|---------|
| `ekf_slam_hls_wrapper.h` | Wrapper class header | ~40 | Interface to use HLS from software |
| `ekf_slam_hls_wrapper.cpp` | Wrapper implementation | ~150 | Data conversion, HLS invocation |

### Testing & Build Files
| File | Description | Purpose |
|------|-------------|---------|
| `testbench_hls.cpp` | HLS testbench | Verify HLS kernel functionality |
| `create_hls_project.tcl` | Vitis HLS script | Automate HLS project creation |
| `HLS_README.md` | Complete documentation | Usage guide and reference |
| `HLS_SUMMARY.md` | This file | Quick reference |

### Updated Files
| File | Changes |
|------|---------|
| `CMakeLists.txt` | Added HLS library and testbench targets |

## Key Features

### ✅ Problems Solved

Your original code had these HLS incompatibilities:
1. ❌ Dynamic memory (`std::vector`) → ✅ Fixed-size arrays
2. ❌ Variable state size → ✅ Fixed max size with tracking
3. ❌ STL containers/algorithms → ✅ Manual implementations
4. ❌ Console I/O → ✅ Removed from kernel
5. ❌ Unbounded loops → ✅ Loop tripcount pragmas
6. ❌ Runtime matrix dimensions → ✅ Compile-time bounds

### 🎯 Design Highlights

**Fixed Size Limits** (adjustable in [ekf_slam_hls.h](ekf_slam_hls.h)):
```cpp
#define MAX_LANDMARKS 50        // Maximum number of landmarks
#define MAX_STATE_SIZE 103      // 3 + 2*50
#define MAX_OBSERVATIONS 10     // Observations per update
```

**HLS Kernel Interface**:
```cpp
void ekf_slam_update_hls(
    EKFStateHLS* state,              // In/Out: Current state
    const EKFParamsHLS* params,      // In: Algorithm parameters
    const double u[2],                // In: Control input
    const ObservationHLS obs[MAX_OBSERVATIONS],  // In: Sensor data
    int num_observations              // In: Number of observations
);
```

**Automatic Fallback**: If limits are exceeded, automatically uses software implementation.

## Usage

### Quick Start

```cpp
#include "ekf_slam_hls_wrapper.h"

// Create HLS-accelerated instance
EKFSLAM_HLS slam;

// Initialize with your data
EKFSLAMDataLoader loader;
loader.loadInitialConditions("ekf_slam_initial_conditions.txt");
slam.initializeHLS(loader.getInitialConditions());

// Use exactly like the original!
double u[2] = {1.0, 0.1};
std::vector<Observation> observations = {...};
slam.update(u, observations);  // Automatically uses FPGA
```

### Building

```bash
# Build everything
cd build
cmake ..
make

# Run original testbench
./ekf_slam_testbench

# Run HLS testbench
./ekf_slam_hls_testbench
```

### HLS Synthesis

```bash
# Run Vitis HLS synthesis
vitis_hls -f create_hls_project.tcl

# Results will be in: ekf_slam_hls_project/solution1/
```

## Next Steps

### 1. Verify Functionality ✓
```bash
cd build
./ekf_slam_hls_testbench
```
This will:
- Test basic HLS kernel operations
- Compare software vs HLS results
- Run with your actual data

### 2. Run HLS Synthesis
```bash
vitis_hls -f create_hls_project.tcl
```
This will:
- Generate RTL (Verilog/VHDL)
- Create synthesis reports
- Estimate resource usage

### 3. Review Synthesis Reports

Check these files in `ekf_slam_hls_project/solution1/syn/report/`:
- `ekf_slam_update_hls_csynth.rpt` - Main synthesis report
- Look for:
  - **Latency**: Cycles to complete one update
  - **Resources**: BRAM, DSP, LUT, FF usage
  - **Clock period**: Verify meets 10ns (100 MHz)

### 4. Optimize (if needed)

Common optimizations:

**Reduce Latency**:
```cpp
#pragma HLS PIPELINE II=1       // Pipeline loops
#pragma HLS DATAFLOW            // Pipeline functions
#pragma HLS UNROLL factor=4     // Unroll loops
```

**Reduce Resource Usage**:
```cpp
#pragma HLS RESOURCE variable=x core=RAM_1P  // Use RAM instead of registers
#pragma HLS ALLOCATION instances=mul limit=10  // Limit multipliers
```

**Improve Throughput**:
```cpp
#pragma HLS ARRAY_PARTITION variable=A cyclic factor=4  // Parallel access
```

### 5. Integration Options

**Option A: Software Integration (Quick)**
- Use `EKFSLAM_HLS` wrapper class
- HLS kernel runs on FPGA
- Data transfer via PCIe (Alveo) or AXI (Zynq)

**Option B: Full Hardware System (Advanced)**
- Export as Vivado IP
- Integrate into larger FPGA design
- Connect to sensors directly on FPGA

## Adjusting Limits

If you need to change the maximum sizes:

1. Edit [ekf_slam_hls.h:7-10](ekf_slam_hls.h#L7-L10):
```cpp
#define MAX_LANDMARKS 100       // Increase for more landmarks
#define MAX_OBSERVATIONS 20     // Increase for more sensors
```

2. Rebuild:
```bash
cd build
make clean
make
```

3. Re-run synthesis:
```bash
vitis_hls -f create_hls_project.tcl
```

**Resource Impact**:
- Doubling `MAX_LANDMARKS`: ~4x BRAM, ~2x latency
- Doubling `MAX_OBSERVATIONS`: ~2x latency

## Performance Expectations

### Estimated Performance (100 MHz, 50 landmarks)

| Operation | Cycles | Time |
|-----------|--------|------|
| Prediction | ~5,000 | 50 μs |
| Single observation update | ~15,000 | 150 μs |
| Full update (5 obs) | ~80,000 | 800 μs |

**Actual performance depends on**:
- Number of landmarks
- Number of observations
- FPGA part and clock frequency
- Optimization pragmas applied

### Expected Speedup

Compared to software (single-threaded):
- **Without optimization**: 2-5x
- **With optimization**: 10-50x
- **With fixed-point**: 20-100x

## Troubleshooting

### Build Issues

**Error: `cannot find -lekf_slam_hls_lib`**
```bash
cd build
rm -rf *
cmake ..
make
```

**Error: undeclared identifier in wrapper**
- Make sure `ekf_slam.h` and `ekf_slam_hls.h` are both included

### Synthesis Issues

**Error: "Cannot determine loop trip count"**
- All loops in [ekf_slam_hls.cpp](ekf_slam_hls.cpp) have tripcount pragmas
- If you modify loops, add: `#pragma HLS loop_tripcount min=X max=Y`

**Error: "Cannot synthesize dynamic memory"**
- Check if you added any `new`, `delete`, `std::vector`
- All data structures must be fixed-size

**Warning: "Timing not met"**
- Reduce clock frequency in `create_hls_project.tcl`
- Add more pipelining pragmas
- Consider smaller `MAX_LANDMARKS`

### Runtime Issues

**Results differ from software**
- Small differences (<1e-6) are normal (floating-point rounding)
- Large differences indicate a bug - enable debug prints
- Use `setHLSEnabled(false)` to compare

**Falls back to software**
- Check if exceeding `MAX_LANDMARKS`
- Check if exceeding `MAX_OBSERVATIONS`
- Verify HLS was initialized with `initializeHLS()`

## Implementation Details

### Matrix Storage

Matrices are stored in **row-major order**:
```
Matrix[i][j] → array[i * cols + j]
```

Example: 3x3 matrix
```
[0 1 2]     [0 1 2 3 4 5 6 7 8]
[3 4 5]  →
[6 7 8]
```

### Data Flow

```
Software                  HLS Kernel                 FPGA
+----------+             +----------+              +------+
| EKFSLAM_ | --Convert-> | ekf_slam_| --Synthesize->| RTL  |
|   HLS    |   to HLS    | update_  |    to HDL     |      |
| wrapper  |   format    |   hls()  |               | IP   |
+----------+             +----------+              +------+
     |                        |                        |
     v                        v                        v
 Vector/Matrix           Fixed arrays              Registers
 std::vector            C-style arrays             + BRAM
```

### Memory Hierarchy

1. **Software** (CPU DDR):
   - `xEst_`, `PEst_` (dynamic size)

2. **Transfer** (PCIe/AXI):
   - `EKFStateHLS` struct
   - Fixed-size arrays

3. **FPGA** (BRAM):
   - `state->xEst[MAX_STATE_SIZE]`
   - `state->PEst[MAX_STATE_SIZE²]`

## Additional Resources

- **Detailed Documentation**: See [HLS_README.md](HLS_README.md)
- **Original Code**: [ekf_slam.cpp](ekf_slam.cpp), [ekf_slam.h](ekf_slam.h)
- **Vitis HLS User Guide**: UG1399
- **HLS Pragmas Reference**: UG1253

## Quick Reference Card

| Task | Command |
|------|---------|
| Build software | `cd build && cmake .. && make` |
| Test HLS kernel | `./build/ekf_slam_hls_testbench` |
| Run synthesis | `vitis_hls -f create_hls_project.tcl` |
| View reports | `open ekf_slam_hls_project/solution1/syn/report/*.rpt` |
| Change limits | Edit `ekf_slam_hls.h` lines 7-10 |
| Disable HLS | `slam.setHLSEnabled(false)` |

## Status

- ✅ HLS kernel created and ready for synthesis
- ✅ Wrapper for software integration
- ✅ Test benches created
- ✅ Build system configured
- ⏳ HLS synthesis (run `vitis_hls -f create_hls_project.tcl`)
- ⏳ FPGA integration (after synthesis)
- ⏳ Performance validation (after FPGA deployment)

---

**You're ready to synthesize!** Start with `vitis_hls -f create_hls_project.tcl` and see the results.
