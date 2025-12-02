#include "ekf_slam_hls_wrapper.h"
#include <iostream>
#include <iomanip>
#include <cmath>

// Simple test to verify HLS kernel functionality
void test_hls_kernel_basic() {
    std::cout << "\n=== Basic HLS Kernel Test ===" << std::endl;

    // Create HLS-accelerated SLAM instance
    EKFSLAM_HLS slam;

    // Create simple initial conditions
    EKFSLAMInitialConditions init;
    init.STATE_SIZE = 3;
    init.LM_SIZE = 2;
    init.DT = 0.1;
    init.MAX_RANGE = 10.0;
    init.M_DIST_TH = 9.0;

    // Initial state [x, y, theta]
    init.initial_xEst = Vector(3);
    init.initial_xEst(0) = 0.0;
    init.initial_xEst(1) = 0.0;
    init.initial_xEst(2) = 0.0;

    // Initial covariance
    init.initial_PEst = Matrix(3, 3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            init.initial_PEst(i, j) = (i == j) ? 0.1 : 0.0;
        }
    }

    // Process noise Q (2x2)
    init.Q[0] = 0.1; init.Q[1] = 0.0;
    init.Q[2] = 0.0; init.Q[3] = 0.1;

    // Observation noise Py (2x2)
    init.Py[0] = 0.01; init.Py[1] = 0.0;
    init.Py[2] = 0.0;  init.Py[3] = 0.01;

    // Initialize HLS kernel
    slam.initializeHLS(init);

    // Test 1: Prediction only (no observations)
    std::cout << "\nTest 1: Prediction step" << std::endl;
    double u1[2] = {1.0, 0.1};  // velocity = 1.0, angular velocity = 0.1
    std::vector<Observation> empty_obs;
    slam.update(u1, empty_obs);

    Vector state1 = slam.getStateEstimate();
    std::cout << "State after prediction: [" << state1(0) << ", "
              << state1(1) << ", " << state1(2) << "]" << std::endl;

    // Test 2: Add first landmark
    std::cout << "\nTest 2: Add first landmark" << std::endl;
    double u2[2] = {1.0, 0.0};
    std::vector<Observation> obs1;
    obs1.push_back(Observation(5.0, 0.5, 0));  // range=5.0, bearing=0.5, id=0
    slam.update(u2, obs1);

    Vector state2 = slam.getStateEstimate();
    std::cout << "State size: " << state2.size() << std::endl;
    std::cout << "Number of landmarks: " << slam.getNumLandmarks() << std::endl;
    if (state2.size() >= 5) {
        std::cout << "First landmark: [" << state2(3) << ", " << state2(4) << "]" << std::endl;
    }

    // Test 3: Update with same landmark
    std::cout << "\nTest 3: Update existing landmark" << std::endl;
    double u3[2] = {1.0, 0.0};
    std::vector<Observation> obs2;
    obs2.push_back(Observation(4.8, 0.48, 0));  // Same landmark, slightly different
    slam.update(u3, obs2);

    Vector state3 = slam.getStateEstimate();
    std::cout << "State size: " << state3.size() << std::endl;
    std::cout << "Number of landmarks: " << slam.getNumLandmarks() << std::endl;

    // Test 4: Add multiple landmarks
    std::cout << "\nTest 4: Add multiple landmarks" << std::endl;
    double u4[2] = {1.0, 0.1};
    std::vector<Observation> obs3;
    obs3.push_back(Observation(6.0, -0.3, 1));  // New landmark
    obs3.push_back(Observation(7.0, 0.8, 2));   // Another new landmark
    slam.update(u4, obs3);

    Vector state4 = slam.getStateEstimate();
    std::cout << "State size: " << state4.size() << std::endl;
    std::cout << "Number of landmarks: " << slam.getNumLandmarks() << std::endl;

    std::cout << "\n=== Test Completed Successfully ===" << std::endl;
}

// Test comparison between software and HLS implementations
void test_sw_vs_hls() {
    std::cout << "\n=== Software vs HLS Comparison Test ===" << std::endl;

    // Create both implementations
    EKFSLAM slam_sw;
    EKFSLAM_HLS slam_hw;

    // Same initial conditions
    EKFSLAMInitialConditions init;
    init.STATE_SIZE = 3;
    init.LM_SIZE = 2;
    init.DT = 0.1;
    init.MAX_RANGE = 10.0;
    init.M_DIST_TH = 9.0;

    init.initial_xEst = Vector(3);
    init.initial_xEst(0) = 0.0;
    init.initial_xEst(1) = 0.0;
    init.initial_xEst(2) = 0.0;

    init.initial_PEst = Matrix(3, 3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            init.initial_PEst(i, j) = (i == j) ? 0.1 : 0.0;
        }
    }

    init.Q[0] = 0.1; init.Q[1] = 0.0;
    init.Q[2] = 0.0; init.Q[3] = 0.1;
    init.Py[0] = 0.01; init.Py[1] = 0.0;
    init.Py[2] = 0.0;  init.Py[3] = 0.01;

    slam_sw.initialize(init);
    slam_hw.initializeHLS(init);

    // Run same sequence on both
    double u[2] = {1.0, 0.1};
    std::vector<Observation> obs;
    obs.push_back(Observation(5.0, 0.5, 0));

    slam_sw.update(u, obs);
    slam_hw.update(u, obs);

    // Compare results
    Vector state_sw = slam_sw.getStateEstimate();
    Vector state_hw = slam_hw.getStateEstimate();

    std::cout << "\nSoftware state: [";
    for (size_t i = 0; i < state_sw.size() && i < 5; i++) {
        std::cout << state_sw(i);
        if (i < state_sw.size() - 1 && i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "Hardware state: [";
    for (size_t i = 0; i < state_hw.size() && i < 5; i++) {
        std::cout << state_hw(i);
        if (i < state_hw.size() - 1 && i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Calculate differences
    double max_diff = 0.0;
    if (state_sw.size() == state_hw.size()) {
        for (size_t i = 0; i < state_sw.size(); i++) {
            double diff = std::abs(state_sw(i) - state_hw(i));
            max_diff = std::max(max_diff, diff);
        }
        std::cout << "\nMaximum difference: " << max_diff << std::endl;

        if (max_diff < 1e-6) {
            std::cout << "✓ Results match within tolerance!" << std::endl;
        } else if (max_diff < 1e-3) {
            std::cout << "⚠ Results close but with minor differences" << std::endl;
        } else {
            std::cout << "✗ Significant difference detected!" << std::endl;
        }
    } else {
        std::cout << "✗ State sizes don't match!" << std::endl;
    }
}

// Test with actual data loader
void test_with_dataloader() {
    std::cout << "\n=== Test with Data Loader ===" << std::endl;

    EKFSLAMDataLoader loader;

    // Load initial conditions
    if (!loader.loadInitialConditions("ekf_slam_initial_conditions.txt")) {
        std::cerr << "Failed to load initial conditions" << std::endl;
        return;
    }

    // Load iteration data
    if (!loader.loadIterationData("ekf_slam_data.csv")) {
        std::cerr << "Failed to load iteration data" << std::endl;
        return;
    }

    loader.printSummary();

    // Create HLS SLAM instance
    EKFSLAM_HLS slam;
    slam.initializeHLS(loader.getInitialConditions());

    // Run first 10 iterations
    int num_iterations = std::min(10, (int)loader.getNumIterations());
    std::cout << "\nRunning " << num_iterations << " iterations..." << std::endl;

    for (int i = 0; i < num_iterations; i++) {
        const EKFSLAMIteration* iter = loader.getIteration(i);
        if (!iter) break;

        // Convert observations
        std::vector<Observation> observations;
        for (const auto& obs : iter->observations) {
            observations.push_back(obs);
        }

        // Run update
        slam.update(iter->u, observations);

        if (i % 5 == 0) {
            std::cout << "Iteration " << i << ": "
                      << slam.getNumLandmarks() << " landmarks, "
                      << "state size = " << slam.getStateEstimate().size() << std::endl;
        }
    }

    Vector final_state = slam.getStateEstimate();
    std::cout << "\nFinal robot pose: ["
              << final_state(0) << ", "
              << final_state(1) << ", "
              << final_state(2) << "]" << std::endl;
    std::cout << "Total landmarks: " << slam.getNumLandmarks() << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "EKF SLAM HLS Testbench" << std::endl;
    std::cout << "======================" << std::endl;

    try {
        // Run basic tests
        test_hls_kernel_basic();

        // Compare software vs hardware
        test_sw_vs_hls();

        // Test with actual data (if available)
        test_with_dataloader();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n=== All Tests Complete ===" << std::endl;
    return 0;
}
