#ifndef EKF_SLAM_HLS_H
#define EKF_SLAM_HLS_H

// HLS Configuration - Adjust these based on your requirements
#define MAX_LANDMARKS 50
#define STATE_SIZE 3
#define LM_SIZE 2
#define MAX_STATE_SIZE (STATE_SIZE + LM_SIZE * MAX_LANDMARKS)
#define MAX_OBSERVATIONS 10

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Fixed-size observation structure (HLS-compatible)
struct ObservationHLS {
    double range;
    double bearing;
    int landmark_id;
};

// Fixed-size state structure for HLS
struct EKFStateHLS {
    double xEst[MAX_STATE_SIZE];
    double PEst[MAX_STATE_SIZE * MAX_STATE_SIZE];  // Flattened covariance matrix
    int current_state_size;  // Actual size being used
    int num_landmarks;
    int landmark_ids[MAX_LANDMARKS];  // Track landmark IDs
    int num_landmark_ids;
};

// Initial conditions for HLS kernel
struct EKFParamsHLS {
    double DT;
    double MAX_RANGE;
    double M_DIST_TH;
    double Q[4];   // 2x2 process noise covariance
    double Py[4];  // 2x2 observation noise covariance
    bool KNOWN_DATA_ASSOCIATION;
};

// Main HLS kernel function
void ekf_slam_update_hls(
    EKFStateHLS* state,
    const EKFParamsHLS* params,
    const double u[2],
    const ObservationHLS observations[MAX_OBSERVATIONS],
    int num_observations
);

// Helper functions (will be inlined in HLS)
namespace hls_utils {

    inline double pi2pi(double angle) {
        #pragma HLS INLINE
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    inline void mat_get(const double* mat, int rows, int cols, int i, int j, double& val) {
        #pragma HLS INLINE
        val = mat[i * cols + j];
    }

    inline void mat_set(double* mat, int rows, int cols, int i, int j, double val) {
        #pragma HLS INLINE
        mat[i * cols + j] = val;
    }

    // Matrix multiplication: C = A * B
    // A is (m x n), B is (n x p), C is (m x p)
    void mat_mul(const double* A, const double* B, double* C,
                 int m, int n, int p);

    // Matrix transpose: B = A'
    void mat_transpose(const double* A, double* B, int rows, int cols);

    // Matrix addition: C = A + B
    void mat_add(const double* A, const double* B, double* C,
                 int rows, int cols);

    // Matrix subtraction: C = A - B
    void mat_sub(const double* A, const double* B, double* C,
                 int rows, int cols);

    // 2x2 matrix inversion
    void mat_inv_2x2(const double A[4], double A_inv[4]);

    // Identity matrix
    void mat_eye(double* I, int n);

    // Get submatrix
    void mat_get_sub(const double* A, double* sub,
                     int A_rows, int A_cols,
                     int row_start, int row_end,
                     int col_start, int col_end);

    // Set submatrix
    void mat_set_sub(double* A, const double* sub,
                     int A_rows, int A_cols,
                     int row_start, int col_start,
                     int sub_rows, int sub_cols);
}

#endif // EKF_SLAM_HLS_H
