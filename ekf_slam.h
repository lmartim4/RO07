#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "ekf_slam_dataloader.h"
#include <vector>
#include <cmath>

/**
 * EKF SLAM Implementation (No Eigen dependencies)
 * Suitable for FPGA implementation
 */

class EKFSLAM {
public:
    EKFSLAM();
    
    /**
     * Initialize EKF SLAM with initial conditions
     */
    void initialize(const EKFSLAMInitialConditions& init);
    
    /**
     * Run one iteration of EKF SLAM
     * @param u Control input [velocity, yaw_rate]
     * @param observations Vector of observations
     */
    void update(const double u[2], const std::vector<Observation>& observations);
    
    /**
     * Get current state estimate
     */
    const Vector& getStateEstimate() const { return xEst_; }
    
    /**
     * Get current covariance estimate
     */
    const Matrix& getCovarianceEstimate() const { return PEst_; }
    
    /**
     * Get number of landmarks in state
     */
    int getNumLandmarks() const;
    
    /**
     * Print current state
     */
    void printState() const;
    
private:
    // State and covariance
    Vector xEst_;
    Matrix PEst_;
    
    // Parameters
    int STATE_SIZE_;
    int LM_SIZE_;
    double DT_;
    double MAX_RANGE_;
    double M_DIST_TH_;
    
    // Noise matrices
    Matrix Q_;  // Process noise (2x2)
    Matrix Py_; // Measurement noise (2x2)
    
    // Landmark association (for known data association)
    std::vector<int> trueLandmarkId_;
    bool KNOWN_DATA_ASSOCIATION_;
    
    // Helper functions
    
    /**
     * Calculate number of landmarks in state vector
     */
    int calcNumLandmarks(const Vector& x) const;
    
    /**
     * Normalize angle to [-pi, pi]
     */
    double pi2pi(double angle) const;
    
    /**
     * Motion model: predict next state from current state and control
     */
    Vector motionModel(const Vector& x, const double u[2]) const;
    
    /**
     * Compute Jacobians of motion model
     * @param x Current state (first 3 elements)
     * @param u Control input
     * @param A Output: Jacobian wrt state
     * @param B Output: Jacobian wrt control
     */
    void jacobMotion(const Vector& x, const double u[2], Matrix& A, Matrix& B) const;
    
    /**
     * Calculate absolute landmark position from robot pose and observation
     */
    void calcLandmarkPosition(const Vector& x, const Observation& obs, 
                              double& lm_x, double& lm_y) const;
    
    /**
     * Get landmark position from state vector
     */
    void getLandmarkFromState(const Vector& x, int ind, double& lm_x, double& lm_y) const;
    
    /**
     * Calculate innovation for a landmark observation
     * @param xEst Current state estimate
     * @param PEst Current covariance estimate
     * @param obs Observation [range, bearing]
     * @param lm_id Landmark index in state
     * @param innov Output: innovation vector (2x1)
     * @param S Output: innovation covariance (2x2)
     * @param H Output: observation Jacobian
     */
    void calcInnovation(const Vector& xEst, const Matrix& PEst, 
                       const Observation& obs, int lm_id,
                       Vector& innov, Matrix& S, Matrix& H) const;
    
    /**
     * Compute Jacobian of observation model
     */
    void jacobH(double q, const double delta[2], const Vector& x, int lm_id, 
                Matrix& H) const;
    
    /**
     * Compute Jacobians for augmenting state (adding new landmark)
     */
    void jacobAugment(const Vector& x, const Observation& obs, 
                     Matrix& Jr, Matrix& Jy) const;
    
    /**
     * Search for corresponding landmark using Mahalanobis distance
     * Returns landmark index or nLM if new landmark
     */
    int searchCorrespondLandmarkId(const Vector& xEst, const Matrix& PEst, 
                                   const Observation& obs) const;
    
    /**
     * Prediction step of EKF
     */
    void prediction(const double u[2]);
    
    /**
     * Update step of EKF for one observation
     */
    void updateObservation(const Observation& obs);
    
    /**
     * Add new landmark to state
     */
    void addNewLandmark(const Observation& obs);
    
    // Matrix operations
    
    /**
     * Matrix multiplication: C = A * B
     */
    Matrix matMul(const Matrix& A, const Matrix& B) const;
    
    /**
     * Matrix transpose
     */
    Matrix matTranspose(const Matrix& A) const;
    
    /**
     * Matrix addition: C = A + B
     */
    Matrix matAdd(const Matrix& A, const Matrix& B) const;
    
    /**
     * Matrix subtraction: C = A - B
     */
    Matrix matSub(const Matrix& A, const Matrix& B) const;
    
    /**
     * Matrix-vector multiplication: y = A * x
     */
    Vector matVecMul(const Matrix& A, const Vector& x) const;
    
    /**
     * Vector addition
     */
    Vector vecAdd(const Vector& a, const Vector& b) const;
    
    /**
     * Matrix inverse (2x2 only for now)
     */
    Matrix matInv2x2(const Matrix& A) const;
    
    /**
     * Matrix inverse using Gaussian elimination (general case)
     */
    Matrix matInv(const Matrix& A) const;
    
    /**
     * Extract submatrix
     */
    Matrix getSubMatrix(const Matrix& A, int row_start, int row_end, 
                       int col_start, int col_end) const;
    
    /**
     * Set submatrix
     */
    void setSubMatrix(Matrix& A, const Matrix& sub, 
                     int row_start, int col_start);
    
    /**
     * Create identity matrix
     */
    Matrix eye(int n) const;
};

#endif // EKF_SLAM_H