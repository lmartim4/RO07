#ifndef EKF_SLAM_DATA_LOADER_EMBEDDED_H
#define EKF_SLAM_DATA_LOADER_EMBEDDED_H

#include "ekf_slam_dataloader.h"
#include "ekf_slam_data.h"
#include "ekf_slam_initial_conditions.h"

class EKFSLAMDataLoaderEmbedded {
public:
    EKFSLAMDataLoaderEmbedded();
    bool loadInitialConditionsFromHeader();
    bool loadIterationDataFromHeader();
    const EKFSLAMInitialConditions& getInitialConditions() const { return initial_conditions_; }
    const std::vector<EKFSLAMIteration>& getIterations() const { return iterations_; }
    const EKFSLAMIteration* getIteration(size_t index) const;
    size_t getNumIterations() const { return iterations_.size(); }
    void printSummary() const;

private:
    EKFSLAMInitialConditions initial_conditions_;
    std::vector<EKFSLAMIteration> iterations_;
    Matrix reconstructMatrix(const std::vector<double>& data, int size);
};

#endif
