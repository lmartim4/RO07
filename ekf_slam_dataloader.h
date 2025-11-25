#ifndef EKF_SLAM_DATA_LOADER_H
#define EKF_SLAM_DATA_LOADER_H

#include <vector>
#include <string>

/**
 * Simple dynamic vector class (replacement for Eigen::VectorXd)
 */
class Vector {
public:
    std::vector<double> data;
    
    Vector() {}
    Vector(size_t size) : data(size, 0.0) {}
    Vector(const std::vector<double>& d) : data(d) {}
    
    size_t size() const { return data.size(); }
    void resize(size_t n) { data.resize(n); }
    
    double& operator()(size_t i) { return data[i]; }
    const double& operator()(size_t i) const { return data[i]; }
    
    double& operator[](size_t i) { return data[i]; }
    const double& operator[](size_t i) const { return data[i]; }
    
    // Get pointer to data (useful for FPGA interfacing)
    double* ptr() { return data.data(); }
    const double* ptr() const { return data.data(); }
};

/**
 * Simple dynamic matrix class (replacement for Eigen::MatrixXd)
 */
class Matrix {
public:
    std::vector<double> data;
    size_t rows_;
    size_t cols_;
    
    Matrix() : rows_(0), cols_(0) {}
    Matrix(size_t r, size_t c) : data(r * c, 0.0), rows_(r), cols_(c) {}
    
    size_t rows() const { return rows_; }
    size_t cols() const { return cols_; }
    size_t size() const { return rows_ * cols_; }
    
    void resize(size_t r, size_t c) {
        rows_ = r;
        cols_ = c;
        data.resize(r * c);
    }
    
    // Access element at (row, col)
    double& operator()(size_t i, size_t j) {
        return data[i * cols_ + j];
    }
    
    const double& operator()(size_t i, size_t j) const {
        return data[i * cols_ + j];
    }
    
    // Get pointer to data (useful for FPGA interfacing)
    double* ptr() { return data.data(); }
    const double* ptr() const { return data.data(); }
    
    // Get data in row-major format as vector
    const std::vector<double>& getData() const { return data; }
};

/**
 * Structure to hold observation data (range, bearing, landmark_id)
 */
struct Observation {
    double range;
    double bearing;
    int landmark_id;
    
    Observation() : range(0.0), bearing(0.0), landmark_id(-1) {}
    Observation(double r, double b, int id) : range(r), bearing(b), landmark_id(id) {}
};

/**
 * Structure to hold one iteration of EKF SLAM data
 */
struct EKFSLAMIteration {
    int iteration;
    double execution_time_sec;
    
    // Control input [velocity, yaw_rate]
    double u[2];
    
    // Observations
    std::vector<Observation> observations;
    
    // State before EKF update
    Vector xEst_before;
    Matrix PEst_before;
    
    // State after EKF update
    Vector xEst_after;
    Matrix PEst_after;
};

/**
 * Structure to hold initial conditions
 */
struct EKFSLAMInitialConditions {
    int STATE_SIZE;
    int LM_SIZE;
    double DT;
    double MAX_RANGE;
    double M_DIST_TH;
    
    // Landmarks [x, y] pairs
    std::vector<double> landmarks;  // Flattened: [x0, y0, x1, y1, ...]
    
    Vector initial_xEst;
    Matrix initial_PEst;
    
    // Process noise matrix Q (2x2)
    double Q[4];  // [Q00, Q01, Q10, Q11]
    
    // Measurement noise matrix Py (2x2)
    double Py[4]; // [Py00, Py01, Py10, Py11]
    
    // Helper functions
    int getNumLandmarks() const { return landmarks.size() / 2; }
    
    void getLandmark(int idx, double& x, double& y) const {
        x = landmarks[idx * 2];
        y = landmarks[idx * 2 + 1];
    }
};

/**
 * Class to load and manage EKF SLAM data from CSV files
 * No external dependencies - suitable for FPGA implementation
 */
class EKFSLAMDataLoader {
public:
    EKFSLAMDataLoader();
    
    /**
     * Load initial conditions from file
     * @param filename Path to initial conditions file
     * @return true if successful, false otherwise
     */
    bool loadInitialConditions(const std::string& filename);
    
    /**
     * Load EKF SLAM iteration data from CSV file
     * @param filename Path to CSV file
     * @return true if successful, false otherwise
     */
    bool loadIterationData(const std::string& filename);
    
    /**
     * Get initial conditions
     */
    const EKFSLAMInitialConditions& getInitialConditions() const { 
        return initial_conditions_; 
    }
    
    /**
     * Get all iteration data
     */
    const std::vector<EKFSLAMIteration>& getIterations() const { 
        return iterations_; 
    }
    
    /**
     * Get specific iteration data
     * @param index Iteration index (0-based)
     * @return Pointer to iteration data, or nullptr if index out of bounds
     */
    const EKFSLAMIteration* getIteration(size_t index) const;
    
    /**
     * Get number of iterations loaded
     */
    size_t getNumIterations() const { return iterations_.size(); }
    
    /**
     * Print summary statistics
     */
    void printSummary() const;
    
private:
    EKFSLAMInitialConditions initial_conditions_;
    std::vector<EKFSLAMIteration> iterations_;
    
    /**
     * Helper function to parse a CSV line
     */
    std::vector<std::string> parseCSVLine(const std::string& line);
    
    /**
     * Helper function to reconstruct matrix from flattened data
     */
    Matrix reconstructMatrix(const std::vector<double>& data, int size);
};

#endif // EKF_SLAM_DATA_LOADER_H