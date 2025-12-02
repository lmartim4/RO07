#ifndef EKF_SLAM_HLS_WRAPPER_H
#define EKF_SLAM_HLS_WRAPPER_H

#include "ekf_slam.h"
#include "ekf_slam_hls.h"

// Wrapper class that uses HLS acceleration
class EKFSLAM_HLS : public EKFSLAM {
public:
    EKFSLAM_HLS();
    ~EKFSLAM_HLS();

    // Use HLS kernel for update function (hides base class version)
    void update(const double u[2], const std::vector<Observation>& observations);

    // Initialize HLS kernel with initial conditions
    void initializeHLS(const EKFSLAMInitialConditions& init);

    // Enable/disable HLS acceleration (for testing/debugging)
    void setHLSEnabled(bool enabled) { hls_enabled_ = enabled; }
    bool isHLSEnabled() const { return hls_enabled_; }

private:
    EKFStateHLS hls_state_;
    EKFParamsHLS hls_params_;
    bool hls_enabled_;
    bool hls_initialized_;

    // Helper functions to convert between software and HLS formats
    void copyStateToHLS();
    void copyStateFromHLS();
    void convertObservationsToHLS(const std::vector<Observation>& observations,
                                   ObservationHLS* hls_obs, int& num_obs);
};

#endif // EKF_SLAM_HLS_WRAPPER_H
