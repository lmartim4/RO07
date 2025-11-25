#include "ekf_slam.h"
#include "ekf_slam_dataloader.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

/**
 * Testbench for EKF SLAM implementation
 * Loads data from CSV and validates against Python reference implementation
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

// Helper to compare two states element-wise
void compareStates(const Vector& cpp_state, const Vector& py_state, 
                   const std::string& name, int iteration, 
                   double& max_error, int& error_count) {
    if (cpp_state.size() != py_state.size()) {
        std::cerr << "ERROR at iteration " << iteration << ": " 
                  << name << " size mismatch! C++=" << cpp_state.size() 
                  << " Python=" << py_state.size() << std::endl;
        error_count++;
        return;
    }
    
    Vector diff = vectorSubtract(cpp_state, py_state);
    double norm = vectorNorm(diff);
    max_error = std::max(max_error, norm);
    
    if (norm > 1e-3) {
        std::cerr << "WARNING at iteration " << iteration << ": " 
                  << name << " difference = " << norm << std::endl;
        
        // Print first few elements for debugging
        std::cerr << "  C++ state (first 5): [";
        for (size_t i = 0; i < std::min((size_t)5, cpp_state.size()); i++) {
            std::cerr << cpp_state(i);
            if (i < std::min((size_t)5, cpp_state.size()) - 1) std::cerr << ", ";
        }
        std::cerr << "]" << std::endl;
        
        std::cerr << "  Python state (first 5): [";
        for (size_t i = 0; i < std::min((size_t)5, py_state.size()); i++) {
            std::cerr << py_state(i);
            if (i < std::min((size_t)5, py_state.size()) - 1) std::cerr << ", ";
        }
        std::cerr << "]" << std::endl;
        
        error_count++;
    }
}

// Helper to compare two covariance matrices
void compareCovariances(const Matrix& cpp_cov, const Matrix& py_cov, 
                       const std::string& name, int iteration,
                       double& max_error, int& error_count) {
    if (cpp_cov.rows() != py_cov.rows() || cpp_cov.cols() != py_cov.cols()) {
        std::cerr << "ERROR at iteration " << iteration << ": " 
                  << name << " size mismatch! C++=" << cpp_cov.rows() << "x" << cpp_cov.cols()
                  << " Python=" << py_cov.rows() << "x" << py_cov.cols() << std::endl;
        error_count++;
        return;
    }
    
    Matrix diff = matrixSubtract(cpp_cov, py_cov);
    double norm = matrixNorm(diff);
    max_error = std::max(max_error, norm);
    
    if (norm > 1e-2) {
        std::cerr << "WARNING at iteration " << iteration << ": " 
                  << name << " difference = " << norm << std::endl;
        error_count++;
    }
}

int main(int argc, char** argv) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "EKF SLAM C++ Implementation Testbench" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Load data
    EKFSLAMDataLoader loader;
    
    std::cout << "Loading initial conditions..." << std::endl;
    if (!loader.loadInitialConditions("ekf_slam_initial_conditions.txt")) {
        std::cerr << "Failed to load initial conditions" << std::endl;
        return 1;
    }
    
    std::cout << "Loading iteration data..." << std::endl;
    if (!loader.loadIterationData("ekf_slam_data.csv")) {
        std::cerr << "Failed to load iteration data" << std::endl;
        return 1;
    }
    
    loader.printSummary();
    
    // Initialize EKF SLAM
    const EKFSLAMInitialConditions& init = loader.getInitialConditions();
    EKFSLAM ekf;
    ekf.initialize(init);
    
    std::cout << "\n=== Initial State ===" << std::endl;
    ekf.printState();
    
    // Validation statistics
    size_t num_iterations = loader.getNumIterations();
    double max_state_error = 0.0;
    double max_covariance_error = 0.0;
    int error_count = 0;
    double total_cpp_time = 0.0;
    double total_py_time = 0.0;
    
    std::cout << "\n=== Running Validation ===" << std::endl;
    std::cout << "Processing " << num_iterations << " iterations..." << std::endl;
    
    // Process each iteration
    for (size_t i = 0; i < num_iterations; i++) {
        const EKFSLAMIteration* iter = loader.getIteration(i);
        if (!iter) {
            std::cerr << "Failed to get iteration " << i << std::endl;
            continue;
        }
        
        // Verify state before update matches
        const Vector& xEst_before = ekf.getStateEstimate();
        compareStates(xEst_before, iter->xEst_before, "State before", iter->iteration,
                     max_state_error, error_count);
        
        // Run EKF SLAM update
        auto start = std::chrono::high_resolution_clock::now();
        ekf.update(iter->u, iter->observations);
        auto end = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double> cpp_time = end - start;
        total_cpp_time += cpp_time.count();
        total_py_time += iter->execution_time_sec;
        
        // Compare results
        const Vector& xEst_after = ekf.getStateEstimate();
        const Matrix& PEst_after = ekf.getCovarianceEstimate();
        
        compareStates(xEst_after, iter->xEst_after, "State after", iter->iteration,
                     max_state_error, error_count);
        compareCovariances(PEst_after, iter->PEst_after, "Covariance after", iter->iteration,
                          max_covariance_error, error_count);
        
        // Print progress
        if ((i + 1) % 100 == 0 || i == 0 || i == num_iterations - 1) {
            std::cout << "  Progress: " << (i + 1) << "/" << num_iterations 
                      << " iterations";
            if (i > 0) {
                std::cout << " (avg time: " 
                          << std::fixed << std::setprecision(3)
                          << (total_cpp_time / (i + 1) * 1000.0) << " ms)";
            }
            std::cout << std::endl;
        }
    }
    
    // Print validation results
    std::cout << "\n========================================" << std::endl;
    std::cout << "Validation Results" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total iterations processed: " << num_iterations << std::endl;
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "Maximum state error: " << max_state_error << std::endl;
    std::cout << "Maximum covariance error: " << max_covariance_error << std::endl;
    std::cout << "Number of warnings/errors: " << error_count << std::endl;
    
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Average C++ execution time: " 
              << (total_cpp_time / num_iterations * 1000.0) << " ms" << std::endl;
    std::cout << "Average Python execution time: " 
              << (total_py_time / num_iterations * 1000.0) << " ms" << std::endl;
    double speedup = total_py_time / total_cpp_time;
    std::cout << "Speedup: " << speedup << "x" << std::endl;
    
    // Final state
    std::cout << "\n=== Final State ===" << std::endl;
    ekf.printState();
    
    // Detailed example: Show iteration 10
    std::cout << "\n========================================" << std::endl;
    std::cout << "Example: Iteration 10 Details" << std::endl;
    std::cout << "========================================" << std::endl;
    const EKFSLAMIteration* iter10 = loader.getIteration(10);
    if (iter10) {
        std::cout << "Iteration: " << iter10->iteration << std::endl;
        std::cout << "Python execution time: " 
                  << std::fixed << std::setprecision(3)
                  << (iter10->execution_time_sec * 1000.0) << " ms" << std::endl;
        std::cout << "Control input u: [" << iter10->u[0] << ", " << iter10->u[1] << "]" << std::endl;
        std::cout << "Number of observations: " << iter10->observations.size() << std::endl;
        
        for (size_t i = 0; i < iter10->observations.size(); i++) {
            const Observation& obs = iter10->observations[i];
            std::cout << "  Obs " << i << ": range=" << obs.range 
                      << ", bearing=" << obs.bearing 
                      << ", id=" << obs.landmark_id << std::endl;
        }
        
        std::cout << "\nState before (first 5 elements): [";
        for (size_t i = 0; i < std::min((size_t)5, iter10->xEst_before.size()); i++) {
            std::cout << iter10->xEst_before(i);
            if (i < std::min((size_t)5, iter10->xEst_before.size()) - 1) std::cout << ", ";
        }
        if (iter10->xEst_before.size() > 5) std::cout << ", ...";
        std::cout << "]" << std::endl;
        
        std::cout << "State after (first 5 elements): [";
        for (size_t i = 0; i < std::min((size_t)5, iter10->xEst_after.size()); i++) {
            std::cout << iter10->xEst_after(i);
            if (i < std::min((size_t)5, iter10->xEst_after.size()) - 1) std::cout << ", ";
        }
        if (iter10->xEst_after.size() > 5) std::cout << ", ...";
        std::cout << "]" << std::endl;
    }
    
    // Summary
    std::cout << "\n========================================" << std::endl;
    if (error_count == 0 && max_state_error < 1e-3 && max_covariance_error < 1e-2) {
        std::cout << "✓ VALIDATION PASSED" << std::endl;
        std::cout << "C++ implementation matches Python reference!" << std::endl;
    } else if (error_count < 10 && max_state_error < 1e-2) {
        std::cout << "⚠ VALIDATION PASSED WITH WARNINGS" << std::endl;
        std::cout << "C++ implementation mostly matches Python reference." << std::endl;
        std::cout << "Small differences may be due to numerical precision." << std::endl;
    } else {
        std::cout << "✗ VALIDATION FAILED" << std::endl;
        std::cout << "C++ implementation differs significantly from Python reference." << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
    
    return 0;
}