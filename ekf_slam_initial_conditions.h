#ifndef EKF_SLAM_INITIAL_CONDITIONS_H
#define EKF_SLAM_INITIAL_CONDITIONS_H

// Auto-generated header file from initial conditions
// Source: ekf_slam_initial_conditions.txt

// EKF SLAM Parameters
#define PARAM_STATE_SIZE 3
#define PARAM_LM_SIZE 2
#define PARAM_DT 0.1f
#define PARAM_MAX_RANGE 10.0f
#define PARAM_M_DIST_TH 9.0f
#define PARAM_NUM_LANDMARKS 4

// Landmark positions [x, y]
static const float landmarks[PARAM_NUM_LANDMARKS][PARAM_LM_SIZE] = {
    {0.0f, 5.0f},
    {11.0f, 1.0f},
    {3.0f, 15.0f},
    {-5.0f, 20.0f}
};

// Initial state estimate (xEst)
static const float initial_xEst[PARAM_STATE_SIZE] = {
    0.0f, 0.0f, 0.0f
};

// Initial state covariance matrix (PEst)
static const float initial_PEst[PARAM_STATE_SIZE][PARAM_STATE_SIZE] = {
    {0.01f, 0.0f, 0.0f},
    {0.0f, 0.01f, 0.0f},
    {0.0f, 0.0f, 0.0001f}
};

// Process noise covariance matrix (Q)
#define PARAM_Q_SIZE 2
static const float Q_matrix[PARAM_Q_SIZE][PARAM_Q_SIZE] = {
    {0.18f, 0.0f},
    {0.0f, 0.005483114f}
};

// Observation noise covariance matrix (Py)
#define PARAM_PY_SIZE 2
static const float Py_matrix[PARAM_PY_SIZE][PARAM_PY_SIZE] = {
    {0.02f, 0.0f},
    {0.0f, 0.01523087f}
};

#endif // EKF_SLAM_INITIAL_CONDITIONS_H
