#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "ekf_slam_dataloader.h"
#include <vector>
#include <cmath>

class EKFSLAM {
public:
    EKFSLAM();
    
    void initialize(const EKFSLAMInitialConditions& init);    
    void update(const double u[2], const std::vector<Observation>& observations);
    const Vector& getStateEstimate() const { return xEst_; }
    const Matrix& getCovarianceEstimate() const { return PEst_; }
    int getNumLandmarks() const;
    void printState() const;
private:
    Vector xEst_;
    Matrix PEst_;
    
    int STATE_SIZE_;
    int LM_SIZE_;
    double DT_;
    double MAX_RANGE_;
    double M_DIST_TH_;
    
    Matrix Q_;
    Matrix Py_;
    
    std::vector<int> trueLandmarkId_;
    bool KNOWN_DATA_ASSOCIATION_;
    
    int calcNumLandmarks(const Vector& x) const;
    
    double pi2pi(double angle) const;
    Vector motionModel(const Vector& x, const double u[2]) const;
    
    void jacobMotion(const Vector& x, const double u[2], Matrix& A, Matrix& B) const;
    void calcLandmarkPosition(const Vector& x, const Observation& obs, double& lm_x, double& lm_y) const;
    void getLandmarkFromState(const Vector& x, int ind, double& lm_x, double& lm_y) const;
    void calcInnovation(const Vector& xEst, const Matrix& PEst, const Observation& obs, int lm_id, Vector& innov, Matrix& S, Matrix& H) const;
    void jacobH(double q, const double delta[2], const Vector& x, int lm_id, Matrix& H) const;
    void jacobAugment(const Vector& x, const Observation& obs, Matrix& Jr, Matrix& Jy) const;
    int searchCorrespondLandmarkId(const Vector& xEst, const Matrix& PEst, const Observation& obs) const;
    void prediction(const double u[2]);
    void updateObservation(const Observation& obs);
    void addNewLandmark(const Observation& obs);
    Matrix matMul(const Matrix& A, const Matrix& B) const;
    Matrix matTranspose(const Matrix& A) const;
    Matrix matAdd(const Matrix& A, const Matrix& B) const;
    Matrix matSub(const Matrix& A, const Matrix& B) const;
    Vector matVecMul(const Matrix& A, const Vector& x) const;
    Vector vecAdd(const Vector& a, const Vector& b) const;
    Matrix matInv2x2(const Matrix& A) const;
    Matrix matInv(const Matrix& A) const;
    Matrix getSubMatrix(const Matrix& A, int row_start, int row_end, int col_start, int col_end) const;
    void setSubMatrix(Matrix& A, const Matrix& sub, int row_start, int col_start);
    Matrix eye(int n) const;
};

#endif