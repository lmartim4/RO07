#ifndef EKF_SLAM_DATA_LOADER_H
#define EKF_SLAM_DATA_LOADER_H

#include <vector>
#include <string>

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
    double* ptr() { return data.data(); }
    const double* ptr() const { return data.data(); }
};

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
    
    double& operator()(size_t i, size_t j) { return data[i * cols_ + j]; }
    const double& operator()(size_t i, size_t j) const { return data[i * cols_ + j]; }
    double* ptr() { return data.data(); }
    const double* ptr() const { return data.data(); }
    const std::vector<double>& getData() const { return data; }
};

struct Observation {
    double range;
    double bearing;
    int landmark_id;
    
    Observation() : range(0.0), bearing(0.0), landmark_id(-1) {}
    Observation(double r, double b, int id) : range(r), bearing(b), landmark_id(id) {}
};

struct EKFSLAMIteration {
    int iteration;
    double execution_time_sec;
    
    double u[2];
    std::vector<Observation> observations;
    
    Vector xEst_before;
    Matrix PEst_before;
    
    Vector xEst_after;
    Matrix PEst_after;
};

struct EKFSLAMInitialConditions {
    int STATE_SIZE;
    int LM_SIZE;
    double DT;
    double MAX_RANGE;
    double M_DIST_TH;
    
    std::vector<double> landmarks;
    
    Vector initial_xEst;
    Matrix initial_PEst;
    
    
    double Q[4];
    double Py[4];
    
    int getNumLandmarks() const { return landmarks.size() / 2; }
    
    void getLandmark(int idx, double& x, double& y) const {
        x = landmarks[idx * 2];
        y = landmarks[idx * 2 + 1];
    }
};

class EKFSLAMDataLoader {
public:
    EKFSLAMDataLoader();
    bool loadInitialConditions(const std::string& filename);
    bool loadIterationData(const std::string& filename);
    const EKFSLAMInitialConditions& getInitialConditions() const { return initial_conditions_; }
    const std::vector<EKFSLAMIteration>& getIterations() const { return iterations_; }
    const EKFSLAMIteration* getIteration(size_t index) const;
    size_t getNumIterations() const { return iterations_.size(); }    
    void printSummary() const;
    
private:
    EKFSLAMInitialConditions initial_conditions_;
    std::vector<EKFSLAMIteration> iterations_;
    std::vector<std::string> parseCSVLine(const std::string& line);
    Matrix reconstructMatrix(const std::vector<double>& data, int size);
};

#endif