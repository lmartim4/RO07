#include "ekf_slam_data_loader_no_eigen.h"
#include <iostream>
#include <cmath>

/**
 * Example usage of EKF SLAM data loader (No Eigen dependencies)
 * Suitable for FPGA implementation
 */

// Helper function to compute vector norm
double vectorNorm(const Vector& v) {
    double sum = 0.0;
    for (size_t i = 0; i < v.size(); i++) {
        sum += v(i) * v(i);
    }
    return std::sqrt(sum);
}

// Helper function to compute matrix Frobenius norm
double matrixNorm(const Matrix& m) {
    double sum = 0.0;
    for (size_t i = 0; i < m.rows(); i++) {
        for (size_t j = 0; j < m.cols(); j++) {
            double val = m(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}

// Helper function to subtract vectors
Vector vectorSubtract(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        std::cerr << "Error: Vector size mismatch" << std::endl;
        return Vector();
    }
    
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); i++) {
        result(i) = a(i) - b(i);
    }
    return result;
}

// Helper function to subtract matrices
Matrix matrixSubtract(const Matrix& a, const Matrix& b) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) {
        std::cerr << "Error: Matrix size mismatch" << std::endl;
        return Matrix();
    }
    
    Matrix result(a.rows(), a.cols());
    for (size_t i = 0; i < a.rows(); i++) {
        for (size_t j = 0; j < a.cols(); j++) {
            result(i, j) = a(i, j) - b(i, j);
        }
    }
    return result;
}

int main(int argc, char** argv) {
    // Create loader
    EKFSLAMDataLoader loader;
    
    // Load initial conditions
    if (!loader.loadInitialConditions("ekf_slam_initial_conditions.txt")) {
        std::cerr << "Failed to load initial conditions" << std::endl;
        return 1;
    }
    
    // Load iteration data
    if (!loader.loadIterationData("ekf_slam_data.csv")) {
        std::cerr << "Failed to load iteration data" << std::endl;
        return 1;
    }
    
    // Print summary
    loader.printSummary();
    
    // Get initial conditions
    const EKFSLAMInitialConditions& init = loader.getInitialConditions();
    
    std::cout << "\n=== Initial Conditions Details ===" << std::endl;
    std::cout << "Landmarks:" << std::endl;
    for (int i = 0; i < init.getNumLandmarks(); i++) {
        double x, y;
        init.getLandmark(i, x, y);
        std::cout << "  Landmark " << i << ": (" << x << ", " << y << ")" << std::endl;
    }
    
    std::cout << "\nQ matrix (process noise):" << std::endl;
    std::cout << "  [" << init.Q[0] << ", " << init.Q[1] << "]" << std::endl;
    std::cout << "  [" << init.Q[2] << ", " << init.Q[3] << "]" << std::endl;
    
    std::cout << "\nPy matrix (measurement noise):" << std::endl;
    std::cout << "  [" << init.Py[0] << ", " << init.Py[1] << "]" << std::endl;
    std::cout << "  [" << init.Py[2] << ", " << init.Py[3] << "]" << std::endl;
    
    std::cout << "\n=== Running Validation ===" << std::endl;
    
    // Initialize your C++ EKF SLAM with initial conditions
    Vector xEst = init.initial_xEst;
    Matrix PEst = init.initial_PEst;
    
    // Iterate through all recorded iterations
    size_t num_iterations = loader.getNumIterations();
    double max_state_error = 0.0;
    double max_covariance_error = 0.0;
    int error_count = 0;
    
    for (size_t i = 0; i < num_iterations; i++) {
        const EKFSLAMIteration* iter = loader.getIteration(i);
        if (!iter) continue;
        
        // Verify that the input state matches what we expect
        Vector state_diff = vectorSubtract(xEst, iter->xEst_before);
        double state_diff_norm = vectorNorm(state_diff);
        
        if (state_diff_norm > 1e-6) {
            std::cerr << "Warning: State mismatch at iteration " << iter->iteration 
                      << ", diff = " << state_diff_norm << std::endl;
            error_count++;
        }
        
        // Here you would call your FPGA-compatible EKF SLAM implementation
        // For this example, we'll just use the Python output to show validation
        
        // In your actual implementation:
        // your_ekf_slam(xEst, PEst, iter->u, iter->observations, init.Q, init.Py);
        
        // Compare results with Python output
        Vector state_error = vectorSubtract(xEst, iter->xEst_after);
        double state_error_norm = vectorNorm(state_error);
        
        Matrix cov_error = matrixSubtract(PEst, iter->PEst_after);
        double cov_error_norm = matrixNorm(cov_error);
        
        max_state_error = std::max(max_state_error, state_error_norm);
        max_covariance_error = std::max(max_covariance_error, cov_error_norm);
        
        // Update to Python result for next iteration comparison
        xEst = iter->xEst_after;
        PEst = iter->PEst_after;
        
        // Print progress
        if ((i + 1) % 100 == 0) {
            std::cout << "Processed " << (i + 1) << "/" << num_iterations 
                      << " iterations" << std::endl;
        }
    }
    
    std::cout << "\n=== Validation Results ===" << std::endl;
    std::cout << "Total iterations processed: " << num_iterations << std::endl;
    std::cout << "Maximum state error: " << max_state_error << std::endl;
    std::cout << "Maximum covariance error: " << max_covariance_error << std::endl;
    std::cout << "State mismatch count: " << error_count << std::endl;
    
    // Example: Access specific iteration data for FPGA testing
    std::cout << "\n=== Example: Iteration 10 Data ===" << std::endl;
    const EKFSLAMIteration* iter10 = loader.getIteration(10);
    if (iter10) {
        std::cout << "Iteration: " << iter10->iteration << std::endl;
        std::cout << "Execution time: " << (iter10->execution_time_sec * 1000.0) << " ms" << std::endl;
        std::cout << "Control input u: [" << iter10->u[0] << ", " << iter10->u[1] << "]" << std::endl;
        std::cout << "Number of observations: " << iter10->observations.size() << std::endl;
        
        for (size_t i = 0; i < iter10->observations.size(); i++) {
            const Observation& obs = iter10->observations[i];
            std::cout << "  Obs " << i << ": range=" << obs.range 
                      << ", bearing=" << obs.bearing 
                      << ", id=" << obs.landmark_id << std::endl;
        }
        
        std::cout << "State before: [";
        for (size_t i = 0; i < std::min((size_t)5, iter10->xEst_before.size()); i++) {
            std::cout << iter10->xEst_before(i);
            if (i < std::min((size_t)5, iter10->xEst_before.size()) - 1) std::cout << ", ";
        }
        if (iter10->xEst_before.size() > 5) std::cout << ", ...";
        std::cout << "]" << std::endl;
        
        std::cout << "State after: [";
        for (size_t i = 0; i < std::min((size_t)5, iter10->xEst_after.size()); i++) {
            std::cout << iter10->xEst_after(i);
            if (i < std::min((size_t)5, iter10->xEst_after.size()) - 1) std::cout << ", ";
        }
        if (iter10->xEst_after.size() > 5) std::cout << ", ...";
        std::cout << "]" << std::endl;
        
        // Show how to access raw data pointers (useful for FPGA)
        std::cout << "\nFPGA Interface Example:" << std::endl;
        std::cout << "  xEst data pointer: " << iter10->xEst_after.ptr() << std::endl;
        std::cout << "  PEst data pointer: " << iter10->PEst_after.ptr() << std::endl;
        std::cout << "  Control u array: {" << iter10->u[0] << ", " << iter10->u[1] << "}" << std::endl;
    }
    
    return 0;
}