#include "ekf_slam_hls_wrapper.h"
#include <iostream>
#include <cstring>
#include <algorithm>

EKFSLAM_HLS::EKFSLAM_HLS()
    : EKFSLAM(), hls_enabled_(true), hls_initialized_(false) {
    // Initialize HLS state
    std::memset(&hls_state_, 0, sizeof(EKFStateHLS));
    std::memset(&hls_params_, 0, sizeof(EKFParamsHLS));

    hls_state_.current_state_size = STATE_SIZE_;
    hls_state_.num_landmarks = 0;
    hls_state_.num_landmark_ids = 0;
}

EKFSLAM_HLS::~EKFSLAM_HLS() {
    // Cleanup if needed
}

void EKFSLAM_HLS::initializeHLS(const EKFSLAMInitialConditions& init) {
    // Call parent initialization
    initialize(init);

    // Initialize HLS parameters
    hls_params_.DT = init.DT;
    hls_params_.MAX_RANGE = init.MAX_RANGE;
    hls_params_.M_DIST_TH = init.M_DIST_TH;
    hls_params_.KNOWN_DATA_ASSOCIATION = true;

    // Copy Q and Py matrices
    for (int i = 0; i < 4; i++) {
        hls_params_.Q[i] = init.Q[i];
        hls_params_.Py[i] = init.Py[i];
    }

    // Copy initial state to HLS format
    copyStateToHLS();

    hls_initialized_ = true;

    std::cout << "HLS kernel initialized:" << std::endl;
    std::cout << "  MAX_LANDMARKS: " << MAX_LANDMARKS << std::endl;
    std::cout << "  MAX_STATE_SIZE: " << MAX_STATE_SIZE << std::endl;
    std::cout << "  MAX_OBSERVATIONS: " << MAX_OBSERVATIONS << std::endl;
}

void EKFSLAM_HLS::copyStateToHLS() {
    int size = xEst_.size();

    // Check size limit
    if (size > MAX_STATE_SIZE) {
        std::cerr << "Warning: State size " << size << " exceeds MAX_STATE_SIZE "
                  << MAX_STATE_SIZE << std::endl;
        size = MAX_STATE_SIZE;
    }

    // Copy state vector
    for (int i = 0; i < size; i++) {
        hls_state_.xEst[i] = xEst_(i);
    }

    // Copy covariance matrix (flattened)
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            hls_state_.PEst[i * MAX_STATE_SIZE + j] = PEst_(i, j);
        }
    }

    // Update metadata
    hls_state_.current_state_size = size;
    hls_state_.num_landmarks = calcNumLandmarks(xEst_);

    // Copy landmark IDs
    hls_state_.num_landmark_ids = std::min((int)trueLandmarkId_.size(), MAX_LANDMARKS);
    for (int i = 0; i < hls_state_.num_landmark_ids; i++) {
        hls_state_.landmark_ids[i] = trueLandmarkId_[i];
    }
}

void EKFSLAM_HLS::copyStateFromHLS() {
    int size = hls_state_.current_state_size;

    // Resize if state grew
    if (size != (int)xEst_.size()) {
        xEst_.resize(size);
        PEst_.resize(size, size);
    }

    // Copy state vector
    for (int i = 0; i < size; i++) {
        xEst_(i) = hls_state_.xEst[i];
    }

    // Copy covariance matrix
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            PEst_(i, j) = hls_state_.PEst[i * MAX_STATE_SIZE + j];
        }
    }

    // Update landmark IDs
    trueLandmarkId_.clear();
    for (int i = 0; i < hls_state_.num_landmark_ids; i++) {
        trueLandmarkId_.push_back(hls_state_.landmark_ids[i]);
    }
}

void EKFSLAM_HLS::convertObservationsToHLS(const std::vector<Observation>& observations,
                                            ObservationHLS* hls_obs, int& num_obs) {
    num_obs = std::min((int)observations.size(), MAX_OBSERVATIONS);

    if ((int)observations.size() > MAX_OBSERVATIONS) {
        std::cerr << "Warning: Number of observations " << observations.size()
                  << " exceeds MAX_OBSERVATIONS " << MAX_OBSERVATIONS
                  << ". Truncating." << std::endl;
    }

    for (int i = 0; i < num_obs; i++) {
        hls_obs[i].range = observations[i].range;
        hls_obs[i].bearing = observations[i].bearing;
        hls_obs[i].landmark_id = observations[i].landmark_id;
    }
}

void EKFSLAM_HLS::update(const double u[2], const std::vector<Observation>& observations) {
    if (!hls_enabled_ || !hls_initialized_) {
        // Fall back to software implementation
        EKFSLAM::update(u, observations);
        return;
    }

    // Check if state size would exceed limit
    int potential_new_landmarks = 0;
    for (const auto& obs : observations) {
        auto it = std::find(trueLandmarkId_.begin(), trueLandmarkId_.end(), obs.landmark_id);
        if (it == trueLandmarkId_.end()) {
            potential_new_landmarks++;
        }
    }

    if (hls_state_.num_landmarks + potential_new_landmarks > MAX_LANDMARKS) {
        std::cerr << "Warning: Would exceed MAX_LANDMARKS. Falling back to software." << std::endl;
        EKFSLAM::update(u, observations);
        return;
    }

    // Copy current state to HLS format
    copyStateToHLS();

    // Convert observations to HLS format
    ObservationHLS hls_observations[MAX_OBSERVATIONS];
    int num_obs;
    convertObservationsToHLS(observations, hls_observations, num_obs);

#ifndef __SYNTHESIS__
    // Debug output (won't be synthesized)
    std::cout << "Calling HLS kernel with " << num_obs << " observations" << std::endl;
    std::cout << "Current state size: " << hls_state_.current_state_size << std::endl;
    std::cout << "Current num landmarks: " << hls_state_.num_landmarks << std::endl;
#endif

    // Call HLS kernel
    ekf_slam_update_hls(&hls_state_, &hls_params_, u, hls_observations, num_obs);

#ifndef __SYNTHESIS__
    std::cout << "HLS kernel completed" << std::endl;
    std::cout << "New state size: " << hls_state_.current_state_size << std::endl;
    std::cout << "New num landmarks: " << hls_state_.num_landmarks << std::endl;
#endif

    // Copy results back from HLS format
    copyStateFromHLS();
}
