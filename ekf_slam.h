#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "ekf_slam_dataloader.h"
#include <vector>
#include <cmath>

class EKFSLAM {
public:
    EKFSLAM();
    void initialize(const EKFSLAMInitialConditions& init);
    
    /**
     * Run one iteration of EKF SLAM
     * @param u Control input [velocity, yaw_rate]
     * @param observations Vector of observations
     */
    void update(const double u[2], const std::vector<Observation>& observations);
    const Vector& getStateEstimate() const { return xEst_; }
    const Matrix& getCovarianceEstimate() const { return PEst_; }
    int getNumLandmarks() const;
    void printState() const;
    static Matrix matInv(const Matrix& A);
    static Matrix matInv2x2(const Matrix& A);
    static double pi2pi(double angle);
    static Vector matVecMul(const Matrix& A, const Vector& x);
    
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
    
    int calcNumLandmarks(const Vector& x) const;
    Vector motionModel(const Vector& x, const double u[2]) const;
    
    /**
     * Compute Jacobians of motion model
     * @param x Current state (first 3 elements)
     * @param u Control input
     * @param A Output: Jacobian wrt state
     * @param B Output: Jacobian wrt control
     */
    void jacobMotion(const Vector& x, const double u[2], Matrix& A, Matrix& B) const;
    
    void calcLandmarkPosition(const Vector& x, const Observation& obs, 
                              double& lm_x, double& lm_y) const;
    
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
    
    void jacobH(double q, const double delta[2], const Vector& x, int lm_id, 
                Matrix& H) const;
    
    void jacobAugment(const Vector& x, const Observation& obs, 
                     Matrix& Jr, Matrix& Jy) const;
    
    int searchCorrespondLandmarkId(const Vector& xEst, const Matrix& PEst, 
                                   const Observation& obs) const;
    
    void prediction(const double u[2]);
    void updateObservation(const Observation& obs);
    void addNewLandmark(const Observation& obs);
    Matrix matMul(const Matrix& A, const Matrix& B) const;
    Matrix matTranspose(const Matrix& A) const;
    Matrix matAdd(const Matrix& A, const Matrix& B) const;
    Matrix matSub(const Matrix& A, const Matrix& B) const;
    Vector vecAdd(const Vector& a, const Vector& b) const;
    Matrix getSubMatrix(const Matrix& A, int row_start, int row_end, 
                       int col_start, int col_end) const;
    void setSubMatrix(Matrix& A, const Matrix& sub, 
                     int row_start, int col_start);
    Matrix eye(int n) const;
};

#endif // EKF_SLAM_H