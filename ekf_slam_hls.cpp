#include "ekf_slam_hls.h"
#include <cmath>

// Helper function implementations
namespace hls_utils {

void mat_mul(const double* A, const double* B, double* C,
             int m, int n, int p) {
    #pragma HLS INLINE off

    MAT_MUL_I: for (int i = 0; i < m; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        MAT_MUL_J: for (int j = 0; j < p; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            double sum = 0.0;
            MAT_MUL_K: for (int k = 0; k < n; k++) {
                #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
                #pragma HLS PIPELINE II=1
                sum += A[i * n + k] * B[k * p + j];
            }
            C[i * p + j] = sum;
        }
    }
}

void mat_transpose(const double* A, double* B, int rows, int cols) {
    #pragma HLS INLINE off

    TRANSPOSE_I: for (int i = 0; i < rows; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        TRANSPOSE_J: for (int j = 0; j < cols; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            B[j * rows + i] = A[i * cols + j];
        }
    }
}

void mat_add(const double* A, const double* B, double* C,
             int rows, int cols) {
    #pragma HLS INLINE off

    ADD_I: for (int i = 0; i < rows; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        ADD_J: for (int j = 0; j < cols; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            C[i * cols + j] = A[i * cols + j] + B[i * cols + j];
        }
    }
}

void mat_sub(const double* A, const double* B, double* C,
             int rows, int cols) {
    #pragma HLS INLINE off

    SUB_I: for (int i = 0; i < rows; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        SUB_J: for (int j = 0; j < cols; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            C[i * cols + j] = A[i * cols + j] - B[i * cols + j];
        }
    }
}

void mat_inv_2x2(const double A[4], double A_inv[4]) {
    #pragma HLS INLINE

    double det = A[0] * A[3] - A[1] * A[2];

    // Use small epsilon to avoid division by zero
    if (det > -1e-10 && det < 1e-10) {
        det = 1e-10;  // Regularization
    }

    double inv_det = 1.0 / det;
    A_inv[0] = A[3] * inv_det;
    A_inv[1] = -A[1] * inv_det;
    A_inv[2] = -A[2] * inv_det;
    A_inv[3] = A[0] * inv_det;
}

void mat_eye(double* I, int n) {
    #pragma HLS INLINE off

    EYE_I: for (int i = 0; i < n; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        EYE_J: for (int j = 0; j < n; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            I[i * n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void mat_get_sub(const double* A, double* sub,
                 int A_rows, int A_cols,
                 int row_start, int row_end,
                 int col_start, int col_end) {
    #pragma HLS INLINE off

    int sub_rows = row_end - row_start;
    int sub_cols = col_end - col_start;

    GET_SUB_I: for (int i = 0; i < sub_rows; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        GET_SUB_J: for (int j = 0; j < sub_cols; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            sub[i * sub_cols + j] = A[(row_start + i) * A_cols + (col_start + j)];
        }
    }
}

void mat_set_sub(double* A, const double* sub,
                 int A_rows, int A_cols,
                 int row_start, int col_start,
                 int sub_rows, int sub_cols) {
    #pragma HLS INLINE off

    SET_SUB_I: for (int i = 0; i < sub_rows; i++) {
        #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
        SET_SUB_J: for (int j = 0; j < sub_cols; j++) {
            #pragma HLS loop_tripcount min=2 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            A[(row_start + i) * A_cols + (col_start + j)] = sub[i * sub_cols + j];
        }
    }
}

} // namespace hls_utils

// Forward declarations for internal functions
static void prediction_hls(EKFStateHLS* state, const EKFParamsHLS* params, const double u[2]);
static void update_observation_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs);
static void motion_model(const double x[3], const double u[2], double DT, double xp[3]);
static void jacob_motion(const double x[3], const double u[2], double DT, double A[9], double B[6]);
static void add_new_landmark_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs);
static void kalman_update_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs, int lm_id);
static int search_landmark_id(const EKFStateHLS* state, int true_id);
static void calc_innovation(const EKFStateHLS* state, const EKFParamsHLS* params,
                           const ObservationHLS& obs, int lm_id,
                           double innov[2], double S[4], double* H, int H_cols);

// Motion model: predict next robot state
static void motion_model(const double x[3], const double u[2], double DT, double xp[3]) {
    #pragma HLS INLINE

    xp[0] = x[0] + u[0] * DT * std::cos(x[2]);
    xp[1] = x[1] + u[0] * DT * std::sin(x[2]);
    xp[2] = hls_utils::pi2pi(x[2] + u[1] * DT);
}

// Jacobian of motion model
static void jacob_motion(const double x[3], const double u[2], double DT, double A[9], double B[6]) {
    #pragma HLS INLINE

    // A matrix (3x3)
    A[0] = 1.0;  A[1] = 0.0;  A[2] = -DT * u[0] * std::sin(x[2]);
    A[3] = 0.0;  A[4] = 1.0;  A[5] = DT * u[0] * std::cos(x[2]);
    A[6] = 0.0;  A[7] = 0.0;  A[8] = 1.0;

    // B matrix (3x2)
    B[0] = DT * std::cos(x[2]);  B[1] = 0.0;
    B[2] = DT * std::sin(x[2]);  B[3] = 0.0;
    B[4] = 0.0;                  B[5] = DT;
}

// Prediction step
static void prediction_hls(EKFStateHLS* state, const EKFParamsHLS* params, const double u[2]) {
    #pragma HLS INLINE off

    int total_size = state->current_state_size;

    // Extract robot state
    double x_robot[3];
    PRED_EXTRACT: for (int i = 0; i < 3; i++) {
        #pragma HLS UNROLL
        x_robot[i] = state->xEst[i];
    }

    // Predict robot state
    double x_robot_pred[3];
    motion_model(x_robot, u, params->DT, x_robot_pred);

    // Update state
    PRED_UPDATE_STATE: for (int i = 0; i < 3; i++) {
        #pragma HLS UNROLL
        state->xEst[i] = x_robot_pred[i];
    }

    // Compute Jacobians
    double A[9], B[6];
    jacob_motion(x_robot, u, params->DT, A, B);

    // Temporary buffers for matrix operations
    double P_robot[9];            // 3x3
    double AP[9];                 // 3x3
    double AT[9];                 // 3x3
    double APAT[9];               // 3x3
    double BQ[6];                 // 3x2
    double BT[6];                 // 2x3
    double BQBT[9];               // 3x3
    double P_robot_new[9];        // 3x3

    // Extract P[0:3, 0:3]
    hls_utils::mat_get_sub(state->PEst, P_robot, total_size, total_size, 0, 3, 0, 3);

    // P_robot_new = A * P_robot * A' + B * Q * B'
    hls_utils::mat_mul(A, P_robot, AP, 3, 3, 3);
    hls_utils::mat_transpose(A, AT, 3, 3);
    hls_utils::mat_mul(AP, AT, APAT, 3, 3, 3);

    hls_utils::mat_mul(B, params->Q, BQ, 3, 2, 2);
    hls_utils::mat_transpose(B, BT, 3, 2);
    hls_utils::mat_mul(BQ, BT, BQBT, 3, 3, 3);

    hls_utils::mat_add(APAT, BQBT, P_robot_new, 3, 3);

    // Update P[0:3, 0:3]
    hls_utils::mat_set_sub(state->PEst, P_robot_new, total_size, total_size, 0, 0, 3, 3);

    // Update cross-covariance terms if landmarks exist
    if (state->num_landmarks > 0) {
        int lm_size = LM_SIZE * state->num_landmarks;
        double P_cross[MAX_STATE_SIZE * 3];  // Max 3 x lm_size
        double AP_cross[MAX_STATE_SIZE * 3]; // Max 3 x lm_size

        // P[0:3, 3:] = A * P[0:3, 3:]
        hls_utils::mat_get_sub(state->PEst, P_cross, total_size, total_size, 0, 3, 3, total_size);
        hls_utils::mat_mul(A, P_cross, AP_cross, 3, 3, lm_size);
        hls_utils::mat_set_sub(state->PEst, AP_cross, total_size, total_size, 0, 3, 3, lm_size);

        // P[3:, 0:3] = P[0:3, 3:]'
        double AP_cross_T[MAX_STATE_SIZE * 3];
        hls_utils::mat_transpose(AP_cross, AP_cross_T, 3, lm_size);
        hls_utils::mat_set_sub(state->PEst, AP_cross_T, total_size, total_size, 3, 0, lm_size, 3);
    }

    // Ensure symmetry
    PRED_SYMMETRY_I: for (int i = 0; i < total_size; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        PRED_SYMMETRY_J: for (int j = i + 1; j < total_size; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            double avg = (state->PEst[i * total_size + j] + state->PEst[j * total_size + i]) / 2.0;
            state->PEst[i * total_size + j] = avg;
            state->PEst[j * total_size + i] = avg;
        }
    }
}

// Search for existing landmark by ID
static int search_landmark_id(const EKFStateHLS* state, int true_id) {
    #pragma HLS INLINE off

    SEARCH_LM: for (int i = 0; i < state->num_landmark_ids; i++) {
        #pragma HLS loop_tripcount min=0 max=MAX_LANDMARKS
        if (state->landmark_ids[i] == true_id) {
            return i;
        }
    }
    return -1;  // Not found
}

// Calculate innovation for observation
static void calc_innovation(const EKFStateHLS* state, const EKFParamsHLS* params,
                           const ObservationHLS& obs, int lm_id,
                           double innov[2], double S[4], double* H, int H_cols) {
    #pragma HLS INLINE off

    // Get landmark position from state
    int lm_idx = STATE_SIZE + LM_SIZE * lm_id;
    double lm_x = state->xEst[lm_idx];
    double lm_y = state->xEst[lm_idx + 1];

    // Delta
    double delta_x = lm_x - state->xEst[0];
    double delta_y = lm_y - state->xEst[1];
    double q = delta_x * delta_x + delta_y * delta_y;
    double sq = std::sqrt(q);

    // Predicted observation
    double z_range = sq;
    double z_bearing = hls_utils::pi2pi(std::atan2(delta_y, delta_x) - state->xEst[2]);

    // Innovation
    innov[0] = obs.range - z_range;
    innov[1] = hls_utils::pi2pi(obs.bearing - z_bearing);

    // Build G matrix (2x5)
    double G[10];
    G[0] = -sq * delta_x / q;
    G[1] = -sq * delta_y / q;
    G[2] = 0.0;
    G[3] = sq * delta_x / q;
    G[4] = sq * delta_y / q;

    G[5] = delta_y / q;
    G[6] = -delta_x / q;
    G[7] = -1.0;
    G[8] = -delta_y / q;
    G[9] = delta_x / q;

    // Build F matrix (5 x total_size) - sparse matrix with identity blocks
    double F[5 * MAX_STATE_SIZE];
    CALC_INNOV_F_INIT: for (int i = 0; i < 5 * H_cols; i++) {
        #pragma HLS loop_tripcount min=15 max=5*MAX_STATE_SIZE
        F[i] = 0.0;
    }

    // F1: I(3x3) at top-left
    F[0 * H_cols + 0] = 1.0;
    F[1 * H_cols + 1] = 1.0;
    F[2 * H_cols + 2] = 1.0;

    // F2: I(2x2) at landmark position
    F[3 * H_cols + lm_idx] = 1.0;
    F[4 * H_cols + (lm_idx + 1)] = 1.0;

    // H = G * F (2x5 * 5xN = 2xN)
    hls_utils::mat_mul(G, F, H, 2, 5, H_cols);

    // S = H * P * H' + Py
    double HP[2 * MAX_STATE_SIZE];
    double HT[MAX_STATE_SIZE * 2];
    double HPHT[4];

    hls_utils::mat_mul(H, state->PEst, HP, 2, H_cols, H_cols);
    hls_utils::mat_transpose(H, HT, 2, H_cols);
    hls_utils::mat_mul(HP, HT, HPHT, 2, H_cols, 2);
    hls_utils::mat_add(HPHT, params->Py, S, 2, 2);
}

// Add new landmark to state
static void add_new_landmark_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs) {
    #pragma HLS INLINE off

    // Check if we have space
    if (state->num_landmarks >= MAX_LANDMARKS) {
        return;  // Cannot add more landmarks
    }

    int old_size = state->current_state_size;
    int new_size = old_size + LM_SIZE;

    // Calculate landmark position
    double lm_x = state->xEst[0] + obs.range * std::cos(state->xEst[2] + obs.bearing);
    double lm_y = state->xEst[1] + obs.range * std::sin(state->xEst[2] + obs.bearing);

    // Add to state vector
    state->xEst[old_size] = lm_x;
    state->xEst[old_size + 1] = lm_y;

    // Compute Jacobians Jr (2x3) and Jy (2x2)
    double Jr[6], Jy[4];
    Jr[0] = 1.0;  Jr[1] = 0.0;  Jr[2] = -obs.range * std::sin(state->xEst[2] + obs.bearing);
    Jr[3] = 0.0;  Jr[4] = 1.0;  Jr[5] = obs.range * std::cos(state->xEst[2] + obs.bearing);

    Jy[0] = std::cos(state->xEst[2] + obs.bearing);
    Jy[1] = -obs.range * std::sin(state->xEst[2] + obs.bearing);
    Jy[2] = std::sin(state->xEst[2] + obs.bearing);
    Jy[3] = obs.range * std::cos(state->xEst[2] + obs.bearing);

    // Extend covariance matrix
    double PEst_new[MAX_STATE_SIZE * MAX_STATE_SIZE];

    // Copy old covariance
    ADD_LM_COPY_I: for (int i = 0; i < old_size; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        ADD_LM_COPY_J: for (int j = 0; j < old_size; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            PEst_new[i * new_size + j] = state->PEst[i * old_size + j];
        }
    }

    // Jr @ PEst[0:3, :]
    double P_top[3 * MAX_STATE_SIZE];
    double Jr_P[2 * MAX_STATE_SIZE];
    hls_utils::mat_get_sub(state->PEst, P_top, old_size, old_size, 0, 3, 0, old_size);
    hls_utils::mat_mul(Jr, P_top, Jr_P, 2, 3, old_size);

    // Bottom and right parts
    ADD_LM_CROSS: for (int i = 0; i < LM_SIZE; i++) {
        #pragma HLS UNROLL
        for (int j = 0; j < old_size; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            PEst_new[(old_size + i) * new_size + j] = Jr_P[i * old_size + j];
            PEst_new[j * new_size + (old_size + i)] = Jr_P[i * old_size + j];
        }
    }

    // Bottom-right corner: Jr @ PEst[0:3, 0:3] @ Jr' + Jy @ Py @ Jy'
    double P_robot[9];
    double Jr_P_robot[6];
    double Jr_T[6];
    double Jr_P_Jr_T[4];
    double Jy_Py[4];
    double Jy_T[4];
    double Jy_Py_Jy_T[4];
    double corner[4];

    hls_utils::mat_get_sub(state->PEst, P_robot, old_size, old_size, 0, 3, 0, 3);
    hls_utils::mat_mul(Jr, P_robot, Jr_P_robot, 2, 3, 3);
    hls_utils::mat_transpose(Jr, Jr_T, 2, 3);
    hls_utils::mat_mul(Jr_P_robot, Jr_T, Jr_P_Jr_T, 2, 3, 2);

    hls_utils::mat_mul(Jy, params->Py, Jy_Py, 2, 2, 2);
    hls_utils::mat_transpose(Jy, Jy_T, 2, 2);
    hls_utils::mat_mul(Jy_Py, Jy_T, Jy_Py_Jy_T, 2, 2, 2);

    hls_utils::mat_add(Jr_P_Jr_T, Jy_Py_Jy_T, corner, 2, 2);

    ADD_LM_CORNER: for (int i = 0; i < LM_SIZE; i++) {
        #pragma HLS UNROLL
        for (int j = 0; j < LM_SIZE; j++) {
            #pragma HLS UNROLL
            PEst_new[(old_size + i) * new_size + (old_size + j)] = corner[i * LM_SIZE + j];
        }
    }

    // Copy new covariance back
    ADD_LM_COPY_BACK_I: for (int i = 0; i < new_size; i++) {
        #pragma HLS loop_tripcount min=5 max=MAX_STATE_SIZE
        ADD_LM_COPY_BACK_J: for (int j = 0; j < new_size; j++) {
            #pragma HLS loop_tripcount min=5 max=MAX_STATE_SIZE
            state->PEst[i * MAX_STATE_SIZE + j] = PEst_new[i * new_size + j];
        }
    }

    // Update state metadata
    state->current_state_size = new_size;
    state->landmark_ids[state->num_landmark_ids] = obs.landmark_id;
    state->num_landmark_ids++;
    state->num_landmarks++;
}

// Kalman update for existing landmark
static void kalman_update_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs, int lm_id) {
    #pragma HLS INLINE off

    int size = state->current_state_size;

    // Calculate innovation
    double innov[2];
    double S[4];
    double H[2 * MAX_STATE_SIZE];

    calc_innovation(state, params, obs, lm_id, innov, S, H, size);

    // Kalman gain: K = P * H' * S^-1
    double HT[MAX_STATE_SIZE * 2];
    double PHT[MAX_STATE_SIZE * 2];
    double S_inv[4];
    double K[MAX_STATE_SIZE * 2];

    hls_utils::mat_transpose(H, HT, 2, size);
    hls_utils::mat_mul(state->PEst, HT, PHT, size, size, 2);
    hls_utils::mat_inv_2x2(S, S_inv);
    hls_utils::mat_mul(PHT, S_inv, K, size, 2, 2);

    // State update: x = x + K * innov
    KALMAN_UPDATE_X: for (int i = 0; i < size; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        #pragma HLS PIPELINE II=1
        double K_innov = K[i * 2 + 0] * innov[0] + K[i * 2 + 1] * innov[1];
        state->xEst[i] += K_innov;
    }

    // Covariance update: P = (I - K * H) * P
    double KH[MAX_STATE_SIZE * MAX_STATE_SIZE];
    double I[MAX_STATE_SIZE * MAX_STATE_SIZE];
    double I_KH[MAX_STATE_SIZE * MAX_STATE_SIZE];
    double P_new[MAX_STATE_SIZE * MAX_STATE_SIZE];

    hls_utils::mat_mul(K, H, KH, size, 2, size);
    hls_utils::mat_eye(I, size);
    hls_utils::mat_sub(I, KH, I_KH, size, size);
    hls_utils::mat_mul(I_KH, state->PEst, P_new, size, size, size);

    // Copy back
    KALMAN_COPY_P_I: for (int i = 0; i < size; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        KALMAN_COPY_P_J: for (int j = 0; j < size; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            state->PEst[i * MAX_STATE_SIZE + j] = P_new[i * size + j];
        }
    }

    // Ensure symmetry
    KALMAN_SYMMETRY_I: for (int i = 0; i < size; i++) {
        #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
        KALMAN_SYMMETRY_J: for (int j = i + 1; j < size; j++) {
            #pragma HLS loop_tripcount min=3 max=MAX_STATE_SIZE
            #pragma HLS PIPELINE II=1
            double avg = (state->PEst[i * MAX_STATE_SIZE + j] + state->PEst[j * MAX_STATE_SIZE + i]) / 2.0;
            state->PEst[i * MAX_STATE_SIZE + j] = avg;
            state->PEst[j * MAX_STATE_SIZE + i] = avg;
        }
    }

    // Normalize angle
    state->xEst[2] = hls_utils::pi2pi(state->xEst[2]);
}

// Update with single observation
static void update_observation_hls(EKFStateHLS* state, const EKFParamsHLS* params, const ObservationHLS& obs) {
    #pragma HLS INLINE off

    int min_id = -1;

    if (params->KNOWN_DATA_ASSOCIATION) {
        // Search for existing landmark by ID
        min_id = search_landmark_id(state, obs.landmark_id);
    } else {
        // TODO: Implement data association search
        // For now, just add as new landmark
        min_id = -1;
    }

    // Add new landmark or update existing
    if (min_id == -1) {
        add_new_landmark_hls(state, params, obs);
    } else {
        kalman_update_hls(state, params, obs, min_id);
    }
}

// Main HLS kernel
void ekf_slam_update_hls(
    EKFStateHLS* state,
    const EKFParamsHLS* params,
    const double u[2],
    const ObservationHLS observations[MAX_OBSERVATIONS],
    int num_observations
) {
    // HLS Interface pragmas
    #pragma HLS INTERFACE m_axi port=state bundle=gmem0 depth=1
    #pragma HLS INTERFACE m_axi port=params bundle=gmem1 depth=1
    #pragma HLS INTERFACE s_axilite port=u bundle=control
    #pragma HLS INTERFACE m_axi port=observations bundle=gmem2 depth=MAX_OBSERVATIONS
    #pragma HLS INTERFACE s_axilite port=num_observations bundle=control
    #pragma HLS INTERFACE s_axilite port=return bundle=control

    // Prediction step
    prediction_hls(state, params, u);

    // Update step for each observation
    UPDATE_OBS_LOOP: for (int i = 0; i < num_observations; i++) {
        #pragma HLS loop_tripcount min=1 max=MAX_OBSERVATIONS
        if (i < num_observations && i < MAX_OBSERVATIONS) {
            update_observation_hls(state, params, observations[i]);
        }
    }
}
