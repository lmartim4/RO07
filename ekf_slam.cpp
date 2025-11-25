#include "ekf_slam.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

EKFSLAM::EKFSLAM() 
    : STATE_SIZE_(3), LM_SIZE_(2), DT_(0.1), MAX_RANGE_(10.0), M_DIST_TH_(9.0),
      KNOWN_DATA_ASSOCIATION_(true) {
}

void EKFSLAM::initialize(const EKFSLAMInitialConditions& init) {
    STATE_SIZE_ = init.STATE_SIZE;
    LM_SIZE_ = init.LM_SIZE;
    DT_ = init.DT;
    MAX_RANGE_ = init.MAX_RANGE;
    M_DIST_TH_ = init.M_DIST_TH;
    
    xEst_ = init.initial_xEst;
    PEst_ = init.initial_PEst;
    
    // Initialize Q matrix (2x2)
    Q_ = Matrix(2, 2);
    Q_(0, 0) = init.Q[0];
    Q_(0, 1) = init.Q[1];
    Q_(1, 0) = init.Q[2];
    Q_(1, 1) = init.Q[3];
    
    // Initialize Py matrix (2x2)
    Py_ = Matrix(2, 2);
    Py_(0, 0) = init.Py[0];
    Py_(0, 1) = init.Py[1];
    Py_(1, 0) = init.Py[2];
    Py_(1, 1) = init.Py[3];
    
    trueLandmarkId_.clear();
}

int EKFSLAM::getNumLandmarks() const {
    return calcNumLandmarks(xEst_);
}

int EKFSLAM::calcNumLandmarks(const Vector& x) const {
    return (x.size() - STATE_SIZE_) / LM_SIZE_;
}

double EKFSLAM::pi2pi(double angle) const {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

Vector EKFSLAM::motionModel(const Vector& x, const double u[2]) const {
    Vector xp(3);
    xp(0) = x(0) + u[0] * DT_ * std::cos(x(2));
    xp(1) = x(1) + u[0] * DT_ * std::sin(x(2));
    xp(2) = pi2pi(x(2) + u[1] * DT_);
    return xp;
}

void EKFSLAM::jacobMotion(const Vector& x, const double u[2], Matrix& A, Matrix& B) const {
    A = Matrix(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = 0.0;
    A(0, 2) = -DT_ * u[0] * std::sin(x(2));
    A(1, 0) = 0.0;
    A(1, 1) = 1.0;
    A(1, 2) = DT_ * u[0] * std::cos(x(2));
    A(2, 0) = 0.0;
    A(2, 1) = 0.0;
    A(2, 2) = 1.0;
    
    B = Matrix(3, 2);
    B(0, 0) = DT_ * std::cos(x(2));
    B(0, 1) = 0.0;
    B(1, 0) = DT_ * std::sin(x(2));
    B(1, 1) = 0.0;
    B(2, 0) = 0.0;
    B(2, 1) = DT_;
}

void EKFSLAM::calcLandmarkPosition(const Vector& x, const Observation& obs, 
                                   double& lm_x, double& lm_y) const {
    lm_x = x(0) + obs.range * std::cos(x(2) + obs.bearing);
    lm_y = x(1) + obs.range * std::sin(x(2) + obs.bearing);
}

void EKFSLAM::getLandmarkFromState(const Vector& x, int ind, 
                                   double& lm_x, double& lm_y) const {
    int idx = STATE_SIZE_ + LM_SIZE_ * ind;
    lm_x = x(idx);
    lm_y = x(idx + 1);
}

void EKFSLAM::calcInnovation(const Vector& xEst, const Matrix& PEst, 
                            const Observation& obs, int lm_id,
                            Vector& innov, Matrix& S, Matrix& H) const {
    double lm_x, lm_y;
    getLandmarkFromState(xEst, lm_id, lm_x, lm_y);
    
    double delta_x = lm_x - xEst(0);
    double delta_y = lm_y - xEst(1);
    double delta[2] = {delta_x, delta_y};
    
    double q = delta_x * delta_x + delta_y * delta_y;
    double z_range = std::sqrt(q);
    double z_bearing = pi2pi(std::atan2(delta_y, delta_x) - xEst(2));
    
    // Innovation
    innov = Vector(2);
    innov(0) = obs.range - z_range;
    innov(1) = pi2pi(obs.bearing - z_bearing);
    
    // Compute Jacobian H
    jacobH(q, delta, xEst, lm_id, H);
    
    // Innovation covariance S = H * P * H' + Py
    Matrix HP = matMul(H, PEst);
    Matrix HT = matTranspose(H);
    Matrix HPHT = matMul(HP, HT);
    S = matAdd(HPHT, Py_);
}

void EKFSLAM::jacobH(double q, const double delta[2], const Vector& x, int lm_id, 
                     Matrix& H) const {
    double sq = std::sqrt(q);
    
    // Build G matrix (2x5)
    Matrix G(2, 5);
    G(0, 0) = -sq * delta[0];
    G(0, 1) = -sq * delta[1];
    G(0, 2) = 0.0;
    G(0, 3) = sq * delta[0];
    G(0, 4) = sq * delta[1];
    
    G(1, 0) = delta[1];
    G(1, 1) = -delta[0];
    G(1, 2) = -q;
    G(1, 3) = -delta[1];
    G(1, 4) = delta[0];
    
    // Scale by 1/q
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 5; j++) {
            G(i, j) /= q;
        }
    }
    
    // Build F matrix
    int nLM = calcNumLandmarks(x);
    int total_size = STATE_SIZE_ + LM_SIZE_ * nLM;
    
    Matrix F(5, total_size);
    // Initialize to zero
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < total_size; j++) {
            F(i, j) = 0.0;
        }
    }
    
    // F1: [I(3x3), 0(3x2*nLM)]
    F(0, 0) = 1.0;
    F(1, 1) = 1.0;
    F(2, 2) = 1.0;
    
    // F2: [0(2x3), 0(2x2*i), I(2x2), 0(2x2*(nLM-i-1))]
    int lm_idx = STATE_SIZE_ + LM_SIZE_ * lm_id;
    F(3, lm_idx) = 1.0;
    F(4, lm_idx + 1) = 1.0;
    
    // H = G * F
    H = matMul(G, F);
}

void EKFSLAM::jacobAugment(const Vector& x, const Observation& obs, 
                          Matrix& Jr, Matrix& Jy) const {
    Jr = Matrix(2, 3);
    Jr(0, 0) = 1.0;
    Jr(0, 1) = 0.0;
    Jr(0, 2) = -obs.range * std::sin(x(2) + obs.bearing);
    Jr(1, 0) = 0.0;
    Jr(1, 1) = 1.0;
    Jr(1, 2) = obs.range * std::cos(x(2) + obs.bearing);
    
    Jy = Matrix(2, 2);
    Jy(0, 0) = std::cos(x(2) + obs.bearing);
    Jy(0, 1) = -obs.range * std::sin(x(2) + obs.bearing);
    Jy(1, 0) = std::sin(x(2) + obs.bearing);
    Jy(1, 1) = obs.range * std::cos(x(2) + obs.bearing);
}

int EKFSLAM::searchCorrespondLandmarkId(const Vector& xEst, const Matrix& PEst, 
                                        const Observation& obs) const {
    int nLM = calcNumLandmarks(xEst);
    
    std::vector<double> min_dist;
    
    for (int i = 0; i < nLM; i++) {
        Vector innov;
        Matrix S, H;
        calcInnovation(xEst, PEst, obs, i, innov, S, H);
        
        // Compute Mahalanobis distance: innov' * S^-1 * innov
        Matrix S_inv = matInv2x2(S);
        
        double dist = 0.0;
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                dist += innov(j) * S_inv(j, k) * innov(k);
            }
        }
        min_dist.push_back(dist);
    }
    
    min_dist.push_back(M_DIST_TH_);  // Threshold for new landmark
    
    // Find minimum
    int min_id = 0;
    double min_val = min_dist[0];
    for (int i = 1; i < (int)min_dist.size(); i++) {
        if (min_dist[i] < min_val) {
            min_val = min_dist[i];
            min_id = i;
        }
    }
    
    return min_id;
}

void EKFSLAM::prediction(const double u[2]) {
    int S = STATE_SIZE_;
    int nLM = calcNumLandmarks(xEst_);
    int total_size = S + LM_SIZE_ * nLM;
    
    // Predict robot state
    Vector x_robot(S);
    for (int i = 0; i < S; i++) {
        x_robot(i) = xEst_(i);
    }
    
    Vector x_robot_pred = motionModel(x_robot, u);
    
    // Update state
    for (int i = 0; i < S; i++) {
        xEst_(i) = x_robot_pred(i);
    }
    
    // Compute Jacobians
    Matrix A, B;
    jacobMotion(x_robot, u, A, B);
    
    // Update covariance
    // P[0:S, 0:S] = A * P[0:S, 0:S] * A' + B * Q * B'
    Matrix P_robot = getSubMatrix(PEst_, 0, S, 0, S);
    Matrix AP = matMul(A, P_robot);
    Matrix AT = matTranspose(A);
    Matrix APAT = matMul(AP, AT);
    
    Matrix BQ = matMul(B, Q_);
    Matrix BT = matTranspose(B);
    Matrix BQBT = matMul(BQ, BT);
    
    Matrix P_robot_new = matAdd(APAT, BQBT);
    setSubMatrix(PEst_, P_robot_new, 0, 0);
    
    // Update cross-covariance terms
    // P[0:S, S:] = A * P[0:S, S:]
    if (nLM > 0) {
        Matrix P_cross = getSubMatrix(PEst_, 0, S, S, total_size);
        Matrix AP_cross = matMul(A, P_cross);
        setSubMatrix(PEst_, AP_cross, 0, S);
        
        // P[S:, 0:S] = P[0:S, S:]'
        Matrix AP_cross_T = matTranspose(AP_cross);
        setSubMatrix(PEst_, AP_cross_T, S, 0);
    }
    
    // Ensure symmetry
    for (int i = 0; i < total_size; i++) {
        for (int j = i + 1; j < total_size; j++) {
            double avg = (PEst_(i, j) + PEst_(j, i)) / 2.0;
            PEst_(i, j) = avg;
            PEst_(j, i) = avg;
        }
    }
}

void EKFSLAM::addNewLandmark(const Observation& obs) {
    // Calculate landmark position
    double lm_x, lm_y;
    calcLandmarkPosition(xEst_, obs, lm_x, lm_y);
    
    // Extend state vector
    int old_size = xEst_.size();
    xEst_.resize(old_size + LM_SIZE_);
    xEst_(old_size) = lm_x;
    xEst_(old_size + 1) = lm_y;
    
    // Compute Jacobians
    Matrix Jr, Jy;
    Vector x_robot(3);
    for (int i = 0; i < 3; i++) {
        x_robot(i) = xEst_(i);
    }
    jacobAugment(x_robot, obs, Jr, Jy);
    
    // Extend covariance matrix
    int new_size = old_size + LM_SIZE_;
    Matrix PEst_new(new_size, new_size);
    
    // Copy old covariance
    for (int i = 0; i < old_size; i++) {
        for (int j = 0; j < old_size; j++) {
            PEst_new(i, j) = PEst_(i, j);
        }
    }
    
    // Bottom part: Jr @ PEst[0:3, :]
    Matrix P_top = getSubMatrix(PEst_, 0, 3, 0, old_size);
    Matrix Jr_P = matMul(Jr, P_top);
    
    // Right part: transpose of bottom part
    for (int i = 0; i < LM_SIZE_; i++) {
        for (int j = 0; j < old_size; j++) {
            PEst_new(old_size + i, j) = Jr_P(i, j);
            PEst_new(j, old_size + i) = Jr_P(i, j);
        }
    }
    
    // Bottom-right corner: Jr @ PEst[0:3, 0:3] @ Jr' + Jy @ Py @ Jy'
    Matrix P_robot = getSubMatrix(PEst_, 0, 3, 0, 3);
    Matrix Jr_P_robot = matMul(Jr, P_robot);
    Matrix Jr_T = matTranspose(Jr);
    Matrix Jr_P_Jr_T = matMul(Jr_P_robot, Jr_T);
    
    Matrix Jy_Py = matMul(Jy, Py_);
    Matrix Jy_T = matTranspose(Jy);
    Matrix Jy_Py_Jy_T = matMul(Jy_Py, Jy_T);
    
    Matrix corner = matAdd(Jr_P_Jr_T, Jy_Py_Jy_T);
    
    for (int i = 0; i < LM_SIZE_; i++) {
        for (int j = 0; j < LM_SIZE_; j++) {
            PEst_new(old_size + i, old_size + j) = corner(i, j);
        }
    }
    
    PEst_ = PEst_new;
}

void EKFSLAM::updateObservation(const Observation& obs) {
    int nLM = calcNumLandmarks(xEst_);
    int min_id;
    
    if (KNOWN_DATA_ASSOCIATION_) {
        // Find if landmark already exists
        auto it = std::find(trueLandmarkId_.begin(), trueLandmarkId_.end(), obs.landmark_id);
        if (it != trueLandmarkId_.end()) {
            min_id = std::distance(trueLandmarkId_.begin(), it);
        } else {
            min_id = nLM;
            trueLandmarkId_.push_back(obs.landmark_id);
        }
    } else {
        min_id = searchCorrespondLandmarkId(xEst_, PEst_, obs);
    }
    
    // Add new landmark if needed
    if (min_id == nLM) {
        std::cout << "New LM" << std::endl;
        addNewLandmark(obs);
    } else {
        // Kalman update
        Vector innov;
        Matrix S, H;
        calcInnovation(xEst_, PEst_, obs, min_id, innov, S, H);
        
        // Kalman gain: K = P * H' * S^-1
        Matrix HT = matTranspose(H);
        Matrix PHT = matMul(PEst_, HT);
        Matrix S_inv = matInv2x2(S);
        Matrix K = matMul(PHT, S_inv);
        
        // State update: x = x + K * innov
        Vector K_innov = matVecMul(K, innov);
        xEst_ = vecAdd(xEst_, K_innov);
        
        // Covariance update: P = (I - K * H) * P
        Matrix KH = matMul(K, H);
        Matrix I = eye(xEst_.size());
        Matrix I_KH = matSub(I, KH);
        PEst_ = matMul(I_KH, PEst_);
        
        // Ensure symmetry
        int size = PEst_.rows();
        for (int i = 0; i < size; i++) {
            for (int j = i + 1; j < size; j++) {
                double avg = (PEst_(i, j) + PEst_(j, i)) / 2.0;
                PEst_(i, j) = avg;
                PEst_(j, i) = avg;
            }
        }
    }
    
    // Normalize angle
    xEst_(2) = pi2pi(xEst_(2));
}

void EKFSLAM::update(const double u[2], const std::vector<Observation>& observations) {
    // Prediction step
    prediction(u);
    
    // Update step for each observation
    for (const auto& obs : observations) {
        updateObservation(obs);
    }
}

void EKFSLAM::printState() const {
    std::cout << "State estimate: [";
    for (size_t i = 0; i < std::min((size_t)5, xEst_.size()); i++) {
        std::cout << xEst_(i);
        if (i < std::min((size_t)5, xEst_.size()) - 1) std::cout << ", ";
    }
    if (xEst_.size() > 5) std::cout << ", ...";
    std::cout << "]" << std::endl;
    std::cout << "Number of landmarks: " << getNumLandmarks() << std::endl;
}

// Matrix operations implementation

Matrix EKFSLAM::matMul(const Matrix& A, const Matrix& B) const {
    if (A.cols() != B.rows()) {
        std::cerr << "Matrix dimension mismatch in multiplication" << std::endl;
        return Matrix();
    }
    
    Matrix C(A.rows(), B.cols());
    for (size_t i = 0; i < A.rows(); i++) {
        for (size_t j = 0; j < B.cols(); j++) {
            double sum = 0.0;
            for (size_t k = 0; k < A.cols(); k++) {
                sum += A(i, k) * B(k, j);
            }
            C(i, j) = sum;
        }
    }
    return C;
}

Matrix EKFSLAM::matTranspose(const Matrix& A) const {
    Matrix AT(A.cols(), A.rows());
    for (size_t i = 0; i < A.rows(); i++) {
        for (size_t j = 0; j < A.cols(); j++) {
            AT(j, i) = A(i, j);
        }
    }
    return AT;
}

Matrix EKFSLAM::matAdd(const Matrix& A, const Matrix& B) const {
    if (A.rows() != B.rows() || A.cols() != B.cols()) {
        std::cerr << "Matrix dimension mismatch in addition" << std::endl;
        return Matrix();
    }
    
    Matrix C(A.rows(), A.cols());
    for (size_t i = 0; i < A.rows(); i++) {
        for (size_t j = 0; j < A.cols(); j++) {
            C(i, j) = A(i, j) + B(i, j);
        }
    }
    return C;
}

Matrix EKFSLAM::matSub(const Matrix& A, const Matrix& B) const {
    if (A.rows() != B.rows() || A.cols() != B.cols()) {
        std::cerr << "Matrix dimension mismatch in subtraction" << std::endl;
        return Matrix();
    }
    
    Matrix C(A.rows(), A.cols());
    for (size_t i = 0; i < A.rows(); i++) {
        for (size_t j = 0; j < A.cols(); j++) {
            C(i, j) = A(i, j) - B(i, j);
        }
    }
    return C;
}

Vector EKFSLAM::matVecMul(const Matrix& A, const Vector& x) const {
    if (A.cols() != x.size()) {
        std::cerr << "Matrix-vector dimension mismatch" << std::endl;
        return Vector();
    }
    
    Vector y(A.rows());
    for (size_t i = 0; i < A.rows(); i++) {
        double sum = 0.0;
        for (size_t j = 0; j < A.cols(); j++) {
            sum += A(i, j) * x(j);
        }
        y(i) = sum;
    }
    return y;
}

Vector EKFSLAM::vecAdd(const Vector& a, const Vector& b) const {
    if (a.size() != b.size()) {
        std::cerr << "Vector size mismatch in addition" << std::endl;
        return Vector();
    }
    
    Vector c(a.size());
    for (size_t i = 0; i < a.size(); i++) {
        c(i) = a(i) + b(i);
    }
    return c;
}

Matrix EKFSLAM::matInv2x2(const Matrix& A) const {
    if (A.rows() != 2 || A.cols() != 2) {
        std::cerr << "matInv2x2 requires 2x2 matrix" << std::endl;
        return Matrix();
    }
    
    double det = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    
    if (std::abs(det) < 1e-10) {
        std::cerr << "Matrix is singular or near-singular" << std::endl;
        return Matrix();
    }
    
    Matrix inv(2, 2);
    inv(0, 0) = A(1, 1) / det;
    inv(0, 1) = -A(0, 1) / det;
    inv(1, 0) = -A(1, 0) / det;
    inv(1, 1) = A(0, 0) / det;
    
    return inv;
}

Matrix EKFSLAM::matInv(const Matrix& A) const {
    int n = A.rows();
    if (n != (int)A.cols()) {
        std::cerr << "Matrix must be square for inversion" << std::endl;
        return Matrix();
    }
    
    // For 2x2, use direct formula
    if (n == 2) {
        return matInv2x2(A);
    }
    
    // Create augmented matrix [A | I]
    std::vector<std::vector<double>> aug(n, std::vector<double>(2 * n));
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i][j] = A(i, j);
        }
        for (int j = 0; j < n; j++) {
            aug[i][n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // Gaussian elimination
    for (int i = 0; i < n; i++) {
        // Find pivot
        int max_row = i;
        for (int k = i + 1; k < n; k++) {
            if (std::abs(aug[k][i]) > std::abs(aug[max_row][i])) {
                max_row = k;
            }
        }
        std::swap(aug[i], aug[max_row]);
        
        // Check for singularity
        if (std::abs(aug[i][i]) < 1e-10) {
            std::cerr << "Matrix is singular" << std::endl;
            return Matrix();
        }
        
        // Scale row
        double pivot = aug[i][i];
        for (int j = 0; j < 2 * n; j++) {
            aug[i][j] /= pivot;
        }
        
        // Eliminate column
        for (int k = 0; k < n; k++) {
            if (k != i) {
                double factor = aug[k][i];
                for (int j = 0; j < 2 * n; j++) {
                    aug[k][j] -= factor * aug[i][j];
                }
            }
        }
    }
    
    // Extract inverse from right half
    Matrix inv(n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inv(i, j) = aug[i][n + j];
        }
    }
    
    return inv;
}

Matrix EKFSLAM::getSubMatrix(const Matrix& A, int row_start, int row_end, 
                             int col_start, int col_end) const {
    int rows = row_end - row_start;
    int cols = col_end - col_start;
    
    Matrix sub(rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sub(i, j) = A(row_start + i, col_start + j);
        }
    }
    return sub;
}

void EKFSLAM::setSubMatrix(Matrix& A, const Matrix& sub, 
                          int row_start, int col_start) {
    for (size_t i = 0; i < sub.rows(); i++) {
        for (size_t j = 0; j < sub.cols(); j++) {
            A(row_start + i, col_start + j) = sub(i, j);
        }
    }
}

Matrix EKFSLAM::eye(int n) const {
    Matrix I(n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            I(i, j) = (i == j) ? 1.0 : 0.0;
        }
    }
    return I;
}