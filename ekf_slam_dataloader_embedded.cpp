#include "ekf_slam_dataloader_embedded.h"
#include <iostream>
#include <algorithm>
#include <cmath>

EKFSLAMDataLoaderEmbedded::EKFSLAMDataLoaderEmbedded() {
}

bool EKFSLAMDataLoaderEmbedded::loadInitialConditionsFromHeader() {
    std::cout << "Loading initial conditions from embedded header..." << std::endl;

    // Load parameters from header defines
    initial_conditions_.STATE_SIZE = PARAM_STATE_SIZE;
    initial_conditions_.LM_SIZE = PARAM_LM_SIZE;
    initial_conditions_.DT = PARAM_DT;
    initial_conditions_.MAX_RANGE = PARAM_MAX_RANGE;
    initial_conditions_.M_DIST_TH = PARAM_M_DIST_TH;

    // Load landmarks
    initial_conditions_.landmarks.clear();
    for (int i = 0; i < PARAM_NUM_LANDMARKS; i++) {
        for (int j = 0; j < PARAM_LM_SIZE; j++) {
            initial_conditions_.landmarks.push_back(static_cast<double>(landmarks[i][j]));
        }
    }

    // Load initial xEst
    initial_conditions_.initial_xEst.resize(PARAM_STATE_SIZE);
    for (int i = 0; i < PARAM_STATE_SIZE; i++) {
        initial_conditions_.initial_xEst(i) = static_cast<double>(initial_xEst[i]);
    }

    // Load initial PEst
    std::vector<double> pest_data;
    for (int i = 0; i < PARAM_STATE_SIZE; i++) {
        for (int j = 0; j < PARAM_STATE_SIZE; j++) {
            pest_data.push_back(static_cast<double>(initial_PEst[i][j]));
        }
    }
    initial_conditions_.initial_PEst = reconstructMatrix(pest_data, PARAM_STATE_SIZE);

    // Load Q matrix
    for (int i = 0; i < PARAM_Q_SIZE; i++) {
        for (int j = 0; j < PARAM_Q_SIZE; j++) {
            int idx = i * PARAM_Q_SIZE + j;
            if (idx < 4) {
                initial_conditions_.Q[idx] = static_cast<double>(Q_matrix[i][j]);
            }
        }
    }

    // Load Py matrix
    for (int i = 0; i < PARAM_PY_SIZE; i++) {
        for (int j = 0; j < PARAM_PY_SIZE; j++) {
            int idx = i * PARAM_PY_SIZE + j;
            if (idx < 4) {
                initial_conditions_.Py[idx] = static_cast<double>(Py_matrix[i][j]);
            }
        }
    }

    std::cout << "Successfully loaded initial conditions from header" << std::endl;
    std::cout << "  STATE_SIZE: " << initial_conditions_.STATE_SIZE << std::endl;
    std::cout << "  LM_SIZE: " << initial_conditions_.LM_SIZE << std::endl;
    std::cout << "  NUM_LANDMARKS: " << PARAM_NUM_LANDMARKS << std::endl;

    return true;
}

bool EKFSLAMDataLoaderEmbedded::loadIterationDataFromHeader() {
    std::cout << "Loading iteration data from embedded header..." << std::endl;
    std::cout << "Number of rows available: " << NUM_DATA_ROWS << std::endl;

    // Parse each row from the embedded data
    for (int row = 0; row < NUM_DATA_ROWS; row++) {
        try {
            EKFSLAMIteration iter;
            int idx = 0;
            int row_len = row_lengths[row];

            // Parse basic info
            iter.iteration = static_cast<int>(csv_data[row][idx++]);
            iter.execution_time_sec = csv_data[row][idx++];

            // Parse control input
            iter.u[0] = csv_data[row][idx++];
            iter.u[1] = csv_data[row][idx++];

            // Parse observations
            int num_obs = static_cast<int>(csv_data[row][idx++]);
            for (int i = 0; i < num_obs; i++) {
                Observation obs;
                obs.range = csv_data[row][idx++];
                obs.bearing = csv_data[row][idx++];
                obs.landmark_id = static_cast<int>(csv_data[row][idx++]);
                iter.observations.push_back(obs);
            }

            // Parse xEst_before
            int xEst_before_size = static_cast<int>(csv_data[row][idx++]);
            iter.xEst_before.resize(xEst_before_size);
            for (int i = 0; i < xEst_before_size; i++) {
                iter.xEst_before(i) = csv_data[row][idx++];
            }

            // Parse PEst_before
            int PEst_before_size = static_cast<int>(csv_data[row][idx++]);
            std::vector<double> PEst_before_data;
            for (int i = 0; i < PEst_before_size * PEst_before_size; i++) {
                PEst_before_data.push_back(csv_data[row][idx++]);
            }
            iter.PEst_before = reconstructMatrix(PEst_before_data, PEst_before_size);

            // Parse xEst_after
            int xEst_after_size = static_cast<int>(csv_data[row][idx++]);
            iter.xEst_after.resize(xEst_after_size);
            for (int i = 0; i < xEst_after_size; i++) {
                iter.xEst_after(i) = csv_data[row][idx++];
            }

            // Parse PEst_after
            int PEst_after_size = static_cast<int>(csv_data[row][idx++]);
            std::vector<double> PEst_after_data;
            for (int i = 0; i < PEst_after_size * PEst_after_size; i++) {
                PEst_after_data.push_back(csv_data[row][idx++]);
            }
            iter.PEst_after = reconstructMatrix(PEst_after_data, PEst_after_size);

            iterations_.push_back(iter);

            // Print progress every 100 iterations
            if ((row + 1) % 100 == 0) {
                std::cout << "Loaded " << (row + 1) << " iterations..." << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error parsing row " << row << ": " << e.what() << std::endl;
            continue;
        }
    }

    std::cout << "Successfully loaded " << iterations_.size() << " iterations from header" << std::endl;
    return !iterations_.empty();
}

const EKFSLAMIteration* EKFSLAMDataLoaderEmbedded::getIteration(size_t index) const {
    if (index >= iterations_.size()) {
        return nullptr;
    }
    return &iterations_[index];
}

void EKFSLAMDataLoaderEmbedded::printSummary() const {
    std::cout << "\n=== EKF SLAM Data Summary (Embedded) ===" << std::endl;
    std::cout << "Initial Conditions:" << std::endl;
    std::cout << "  STATE_SIZE: " << initial_conditions_.STATE_SIZE << std::endl;
    std::cout << "  LM_SIZE: " << initial_conditions_.LM_SIZE << std::endl;
    std::cout << "  DT: " << initial_conditions_.DT << std::endl;
    std::cout << "  MAX_RANGE: " << initial_conditions_.MAX_RANGE << std::endl;
    std::cout << "  M_DIST_TH: " << initial_conditions_.M_DIST_TH << std::endl;
    std::cout << "  Number of landmarks: " << initial_conditions_.getNumLandmarks() << std::endl;

    std::cout << "\nIteration Data (from embedded header):" << std::endl;
    std::cout << "  Total iterations: " << iterations_.size() << std::endl;

    if (!iterations_.empty()) {
        double total_time = 0.0;
        double max_time = 0.0;
        double min_time = iterations_[0].execution_time_sec;

        for (const auto& iter : iterations_) {
            total_time += iter.execution_time_sec;
            max_time = std::max(max_time, iter.execution_time_sec);
            min_time = std::min(min_time, iter.execution_time_sec);
        }

        std::cout << "  Average execution time: " << (total_time / iterations_.size() * 1000.0)
                  << " ms" << std::endl;
        std::cout << "  Min execution time: " << (min_time * 1000.0) << " ms" << std::endl;
        std::cout << "  Max execution time: " << (max_time * 1000.0) << " ms" << std::endl;

        std::cout << "\nFirst iteration state sizes:" << std::endl;
        std::cout << "  xEst size: " << iterations_[0].xEst_after.size() << std::endl;
        std::cout << "  PEst size: " << iterations_[0].PEst_after.rows() << "x"
                  << iterations_[0].PEst_after.cols() << std::endl;

        std::cout << "\nLast iteration state sizes:" << std::endl;
        std::cout << "  xEst size: " << iterations_.back().xEst_after.size() << std::endl;
        std::cout << "  PEst size: " << iterations_.back().PEst_after.rows() << "x"
                  << iterations_.back().PEst_after.cols() << std::endl;
    }
    std::cout << "============================\n" << std::endl;
}

Matrix EKFSLAMDataLoaderEmbedded::reconstructMatrix(const std::vector<double>& data, int size) {
    Matrix mat(size, size);

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            mat(i, j) = data[i * size + j];
        }
    }

    return mat;
}
