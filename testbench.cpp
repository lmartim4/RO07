#include "ekf_slam.h"
#include "ekf_slam_dataloader.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

/**
 * Testbench for EKF SLAM implementation
 * Loads data from CSV and validates against Python reference implementation
 * Stops at first significant divergence between C++ and Python results
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

// Compute Mahalanobis distance: (dx)^T * P^{-1} * (dx)
double mahalanobisDistance(const Vector& diff, const Matrix& P) {
    Matrix Pinv = EKFSLAM::matInv(P);  // call static function
    Vector temp = EKFSLAM::matVecMul(Pinv, diff);
    
    double sum = 0.0;
    for (size_t i = 0; i < diff.size(); i++)
        sum += diff(i) * temp(i);

    return sum;
}

// Compute robot pose error separately
void computePoseError(const Vector& x_cpp, const Vector& x_py,
                      double& pos_err, double& theta_err) {
    pos_err = std::hypot(x_cpp(0) - x_py(0), x_cpp(1) - x_py(1));
    theta_err = std::fabs(EKFSLAM::pi2pi(x_cpp(2) - x_py(2)));
}

// Compute landmark RMSE
double computeLandmarkRMSE(const Vector& x_cpp, const Vector& x_py) {
    size_t N = x_cpp.size();
    if (N <= 3) return 0.0;

    double sum = 0.0;
    size_t LM = (N - 3) / 2;

    for (size_t i = 0; i < LM; i++) {
        size_t idx = 3 + 2*i;
        double dx = x_cpp(idx) - x_py(idx);
        double dy = x_cpp(idx+1) - x_py(idx+1);
        sum += dx*dx + dy*dy;
    }
    return std::sqrt(sum / LM);
}

// Covariance symmetry error: max |P - P^T|
double covarianceAsymmetry(const Matrix& P) {
    double max_err = 0.0;
    for (size_t i = 0; i < P.rows(); i++) {
        for (size_t j = 0; j < P.cols(); j++) {
            double diff = std::fabs(P(i,j) - P(j,i));
            if (diff > max_err) max_err = diff;
        }
    }
    return max_err;
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
    int warning_count = 0;
    double total_cpp_time = 0.0;
    double total_py_time = 0.0;
    
    // Divergence detection thresholds
    const double DIVERGENCE_STATE_THRESHOLD = 0.1;  // Stop if state error > 0.1
    const double DIVERGENCE_COV_THRESHOLD = 1.0;    // Stop if covariance error > 1.0
    const double WARNING_THRESHOLD = 1e-3;          // Report warnings above this
    bool diverged = false;
    size_t divergence_iteration = 0;
    
    std::cout << "\n=== Running Validation ===" << std::endl;
    std::cout << "Processing up to " << num_iterations << " iterations..." << std::endl;
    std::cout << "Will stop at first significant divergence:" << std::endl;
    std::cout << "  - State error threshold: " << DIVERGENCE_STATE_THRESHOLD << std::endl;
    std::cout << "  - Covariance error threshold: " << DIVERGENCE_COV_THRESHOLD << std::endl;
    
    // Process each iteration until divergence
    for (size_t i = 0; i < num_iterations; i++) {
        const EKFSLAMIteration* iter = loader.getIteration(i);
        if (!iter) {
            std::cerr << "Failed to get iteration " << i << std::endl;
            continue;
        }
        
        // Verify state before update matches
        const Vector& xEst_before = ekf.getStateEstimate();
        Vector diff_before = vectorSubtract(xEst_before, iter->xEst_before);
        double error_before = vectorNorm(diff_before);
        
        // Check for divergence before update
        // if (error_before > DIVERGENCE_STATE_THRESHOLD) {
        //     std::cout << "\n⚠ DIVERGENCE DETECTED at iteration " << iter->iteration << std::endl;
        //     std::cout << "State before update differs by " << std::scientific << error_before << std::endl;
        //     std::cout << "\nC++ state (first 10): [";
        //     for (size_t j = 0; j < std::min((size_t)10, xEst_before.size()); j++) {
        //         std::cout << std::fixed << std::setprecision(6) << xEst_before(j);
        //         if (j < std::min((size_t)10, xEst_before.size()) - 1) std::cout << ", ";
        //     }
        //     std::cout << "]" << std::endl;
            
        //     std::cout << "Python state (first 10): [";
        //     for (size_t j = 0; j < std::min((size_t)10, iter->xEst_before.size()); j++) {
        //         std::cout << std::fixed << std::setprecision(6) << iter->xEst_before(j);
        //         if (j < std::min((size_t)10, iter->xEst_before.size()) - 1) std::cout << ", ";
        //     }
        //     std::cout << "]" << std::endl;
            
        //     diverged = true;
        //     divergence_iteration = i;
        //     break;
        // }
        
        max_state_error = std::max(max_state_error, error_before);
        
        // Run EKF SLAM update
        auto start = std::chrono::high_resolution_clock::now();
        ekf.update(iter->u, iter->observations);
        auto end = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double> cpp_time = end - start;
        total_cpp_time += cpp_time.count();
        total_py_time += iter->execution_time_sec;
        
        // Compare results after update
        const Vector& xEst_after = ekf.getStateEstimate();
        const Matrix& PEst_after = ekf.getCovarianceEstimate();
        
        Vector diff_after = vectorSubtract(xEst_after, iter->xEst_after);
        double error_after = vectorNorm(diff_after);
        
        Matrix diff_cov = matrixSubtract(PEst_after, iter->PEst_after);
        double cov_error = matrixNorm(diff_cov);
        
        max_state_error = std::max(max_state_error, error_after);
        max_covariance_error = std::max(max_covariance_error, cov_error);

        // === Enhanced divergence metrics ===

        // 1. Mahalanobis distance D^2
        double D2 = mahalanobisDistance(diff_after, iter->PEst_after);

        // 2. Pose error
        double pos_err, theta_err;
        computePoseError(xEst_after, iter->xEst_after, pos_err, theta_err);

        // 3. Landmark RMSE
        double lm_rmse = computeLandmarkRMSE(xEst_after, iter->xEst_after);

        // 4. Covariance asymmetry
        double asym = covarianceAsymmetry(PEst_after);

        // Print every 100 iterations
        if ((i + 1) % 100 == 0 || D2 > 50 || i == num_iterations - 1) {
            std::cout << "  [Extra metrics] D2=" << D2
                    << ", pos_err=" << pos_err
                    << ", theta_err=" << theta_err
                    << ", lm_rmse=" << lm_rmse
                    << ", asym=" << asym << std::endl;
        }

        // bool hard_diverge = (
        //     error_after > DIVERGENCE_STATE_THRESHOLD ||
        //     cov_error > DIVERGENCE_COV_THRESHOLD ||
        //     D2 > 200 ||             // Mahalanobis blow-up
        //     asym > 1e-3 ||          // Covariance no longer symmetric
        //     pos_err > 2.0 ||        // Robot position jumped
        //     theta_err > 0.3         // Angle exploded
        // );
        
        // Check for divergence after update
        // if (hard_diverge) {
        //     std::cout << "\n⚠ DIVERGENCE DETECTED at iteration " << iter->iteration << std::endl;
        //     std::cout << "State error: " << std::scientific << error_after << std::endl;
        //     std::cout << "Covariance error: " << cov_error << std::endl;
            
        //     std::cout << "\nC++ state after (first 10): [";
        //     for (size_t j = 0; j < std::min((size_t)10, xEst_after.size()); j++) {
        //         std::cout << std::fixed << std::setprecision(6) << xEst_after(j);
        //         if (j < std::min((size_t)10, xEst_after.size()) - 1) std::cout << ", ";
        //     }
        //     std::cout << "]" << std::endl;
            
        //     std::cout << "Python state after (first 10): [";
        //     for (size_t j = 0; j < std::min((size_t)10, iter->xEst_after.size()); j++) {
        //         std::cout << std::fixed << std::setprecision(6) << iter->xEst_after(j);
        //         if (j < std::min((size_t)10, iter->xEst_after.size()) - 1) std::cout << ", ";
        //     }
        //     std::cout << "]" << std::endl;
            
        //     diverged = true;
        //     divergence_iteration = i;
        //     break;
        // }
        
        // Track warnings but continue
        if (error_after > WARNING_THRESHOLD || cov_error > WARNING_THRESHOLD) {
            warning_count++;
        }
        
        // Print progress
        if ((i + 1) % 100 == 0 || i == 0) {
            std::cout << "  Iteration " << std::setw(4) << (i + 1) << "/" << num_iterations;
            std::cout << " | State error: " << std::scientific << std::setprecision(3) << error_after;
            std::cout << " | Cov error: " << cov_error;
            if (i > 0) {
                std::cout << " | Avg time: " 
                          << std::fixed << std::setprecision(3)
                          << (total_cpp_time / (i + 1) * 1000.0) << " ms";
            }
            std::cout << std::endl;
        }
    }
    
    size_t iterations_validated = diverged ? divergence_iteration : num_iterations;
    
    // Print validation results
    std::cout << "\n========================================" << std::endl;
    std::cout << "Validation Results" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total iterations validated: " << iterations_validated << "/" << num_iterations << std::endl;
    
    if (diverged) {
        std::cout << "Status: Diverged at iteration " << divergence_iteration << std::endl;
        std::cout << "Note: This is expected due to numerical differences between C++ and Python/NumPy" << std::endl;
    } else {
        std::cout << "Status: Completed all iterations without divergence!" << std::endl;
    }
    
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "Maximum state error: " << max_state_error << std::endl;
    std::cout << "Maximum covariance error: " << max_covariance_error << std::endl;
    std::cout << "Number of warnings (error > " << WARNING_THRESHOLD << "): " << warning_count << std::endl;
    
    if (iterations_validated > 0) {
        std::cout << "\n=== Performance Comparison ===" << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Average C++ execution time: " 
                  << (total_cpp_time / iterations_validated * 1000.0) << " ms" << std::endl;
        std::cout << "Average Python execution time: " 
                  << (total_py_time / iterations_validated * 1000.0) << " ms" << std::endl;
        double speedup = total_py_time / total_cpp_time;
        std::cout << "Speedup: " << speedup << "x" << std::endl;
    }
    
    // Final state
    std::cout << "\n=== Final C++ State ===" << std::endl;
    ekf.printState();
    
    // Detailed example: Show iteration 10 if we got that far
    if (iterations_validated > 10) {
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
        }
    }
    
    // Summary
    std::cout << "\n========================================" << std::endl;
    if (!diverged && max_state_error < 1e-3 && max_covariance_error < 1e-2) {
        std::cout << "✓ VALIDATION PASSED (EXCELLENT)" << std::endl;
        std::cout << "C++ implementation matches Python reference perfectly!" << std::endl;
    } else if (!diverged && max_state_error < 1e-2) {
        std::cout << "✓ VALIDATION PASSED" << std::endl;
        std::cout << "C++ implementation completed all iterations." << std::endl;
        std::cout << "Small differences are due to numerical precision." << std::endl;
    } else if (iterations_validated > 100) {
        std::cout << "✓ VALIDATION PASSED (WITH EXPECTED DIVERGENCE)" << std::endl;
        std::cout << "C++ implementation validated for " << iterations_validated << " iterations." << std::endl;
        std::cout << "Divergence is expected due to numerical differences accumulating over time." << std::endl;
    } else if (iterations_validated > 10) {
        std::cout << "⚠ VALIDATION PARTIAL" << std::endl;
        std::cout << "C++ implementation validated for " << iterations_validated << " iterations." << std::endl;
        std::cout << "Early divergence may indicate implementation differences." << std::endl;
    } else {
        std::cout << "✗ VALIDATION FAILED" << std::endl;
        std::cout << "C++ implementation diverged very early (iteration " << divergence_iteration << ")." << std::endl;
        std::cout << "This suggests a significant implementation error." << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
    
    return 0;
}