#include "ekf_slam_dataloader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

EKFSLAMDataLoader::EKFSLAMDataLoader() {
}

bool EKFSLAMDataLoader::loadInitialConditions(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }
    
    std::string line;
    std::string current_section = "";
    std::string prev_section = "";
    std::vector<double> temp_data;
    
    // Helper lambda to process completed section
    auto processSectionData = [&](const std::string& section) {
        if (temp_data.empty() || section.empty()) return;
        
        if (section == "landmarks") {
            initial_conditions_.landmarks = temp_data;
        } else if (section == "xest") {
            initial_conditions_.initial_xEst = Vector(temp_data);
        } else if (section == "pest") {
            int size = initial_conditions_.STATE_SIZE;
            initial_conditions_.initial_PEst = reconstructMatrix(temp_data, size);
        } else if (section == "q") {
            for (int i = 0; i < 4 && i < (int)temp_data.size(); i++) {
                initial_conditions_.Q[i] = temp_data[i];
            }
        } else if (section == "py") {
            for (int i = 0; i < 4 && i < (int)temp_data.size(); i++) {
                initial_conditions_.Py[i] = temp_data[i];
            }
        }
    };
    
    while (std::getline(file, line)) {
        // Skip empty lines
        if (line.empty()) continue;
        
        // Check for section headers
        if (line.find("# Initial Conditions") != std::string::npos) {
            processSectionData(current_section);
            current_section = "params";
            temp_data.clear();
            continue;
        } else if (line.find("# Landmarks") != std::string::npos) {
            processSectionData(current_section);
            current_section = "landmarks";
            temp_data.clear();
            continue;
        } else if (line.find("# Initial xEst") != std::string::npos) {
            processSectionData(current_section);
            current_section = "xest";
            temp_data.clear();
            continue;
        } else if (line.find("# Initial PEst") != std::string::npos) {
            processSectionData(current_section);
            current_section = "pest";
            temp_data.clear();
            continue;
        } else if (line.find("# Q matrix") != std::string::npos) {
            processSectionData(current_section);
            current_section = "q";
            temp_data.clear();
            continue;
        } else if (line.find("# Py matrix") != std::string::npos) {
            processSectionData(current_section);
            current_section = "py";
            temp_data.clear();
            continue;
        }
        
        // Skip other comment lines
        if (line[0] == '#') continue;
        
        // Parse parameters
        if (current_section == "params") {
            size_t eq_pos = line.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = line.substr(0, eq_pos);
                double value = std::stod(line.substr(eq_pos + 1));
                
                if (key == "STATE_SIZE") initial_conditions_.STATE_SIZE = static_cast<int>(value);
                else if (key == "LM_SIZE") initial_conditions_.LM_SIZE = static_cast<int>(value);
                else if (key == "DT") initial_conditions_.DT = value;
                else if (key == "MAX_RANGE") initial_conditions_.MAX_RANGE = value;
                else if (key == "M_DIST_TH") initial_conditions_.M_DIST_TH = value;
            }
        }
        // Parse numeric data
        else if (current_section == "landmarks" || current_section == "xest" || 
                 current_section == "pest" || current_section == "q" || current_section == "py") {
            std::istringstream iss(line);
            double value;
            while (iss >> value) {
                temp_data.push_back(value);
            }
        }
    }
    
    // Process final section
    processSectionData(current_section);
    
    file.close();
    return true;
}

bool EKFSLAMDataLoader::loadIterationData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }
    
    std::string line;
    // Skip first 2 header lines
    std::getline(file, line); // Skip line 1
    std::getline(file, line); // Skip line 2
    
    // Parse data lines
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::vector<std::string> tokens = parseCSVLine(line);
        if (tokens.empty()) continue;
        
        try {
            EKFSLAMIteration iter;
            size_t idx = 0;
            
            // Parse basic info
            iter.iteration = std::stoi(tokens[idx++]);
            iter.execution_time_sec = std::stod(tokens[idx++]);
            
            // Parse control input
            iter.u[0] = std::stod(tokens[idx++]);
            iter.u[1] = std::stod(tokens[idx++]);
            
            // Parse observations
            int num_obs = std::stoi(tokens[idx++]);
            for (int i = 0; i < num_obs; i++) {
                Observation obs;
                obs.range = std::stod(tokens[idx++]);
                obs.bearing = std::stod(tokens[idx++]);
                obs.landmark_id = std::stoi(tokens[idx++]);
                iter.observations.push_back(obs);
            }
            
            // Parse xEst_before
            int xEst_before_size = std::stoi(tokens[idx++]);
            iter.xEst_before.resize(xEst_before_size);
            for (int i = 0; i < xEst_before_size; i++) {
                iter.xEst_before(i) = std::stod(tokens[idx++]);
            }
            
            // Parse PEst_before
            int PEst_before_size = std::stoi(tokens[idx++]);
            std::vector<double> PEst_before_data;
            for (int i = 0; i < PEst_before_size * PEst_before_size; i++) {
                PEst_before_data.push_back(std::stod(tokens[idx++]));
            }
            iter.PEst_before = reconstructMatrix(PEst_before_data, PEst_before_size);
            
            // Parse xEst_after
            int xEst_after_size = std::stoi(tokens[idx++]);
            iter.xEst_after.resize(xEst_after_size);
            for (int i = 0; i < xEst_after_size; i++) {
                iter.xEst_after(i) = std::stod(tokens[idx++]);
            }
            
            // Parse PEst_after
            int PEst_after_size = std::stoi(tokens[idx++]);
            std::vector<double> PEst_after_data;
            for (int i = 0; i < PEst_after_size * PEst_after_size; i++) {
                PEst_after_data.push_back(std::stod(tokens[idx++]));
            }
            iter.PEst_after = reconstructMatrix(PEst_after_data, PEst_after_size);
            
            iterations_.push_back(iter);
            
        } catch (const std::exception& e) {
            std::cerr << "Error parsing line: " << e.what() << std::endl;
            continue;
        }
    }
    
    file.close();
    return !iterations_.empty();
}

const EKFSLAMIteration* EKFSLAMDataLoader::getIteration(size_t index) const {
    if (index >= iterations_.size()) {
        return nullptr;
    }
    return &iterations_[index];
}

void EKFSLAMDataLoader::printSummary() const {
    std::cout << "\n=== EKF SLAM Data Summary ===" << std::endl;
    std::cout << "Initial Conditions:" << std::endl;
    std::cout << "  STATE_SIZE: " << initial_conditions_.STATE_SIZE << std::endl;
    std::cout << "  LM_SIZE: " << initial_conditions_.LM_SIZE << std::endl;
    std::cout << "  DT: " << initial_conditions_.DT << std::endl;
    std::cout << "  MAX_RANGE: " << initial_conditions_.MAX_RANGE << std::endl;
    std::cout << "  M_DIST_TH: " << initial_conditions_.M_DIST_TH << std::endl;
    std::cout << "  Number of landmarks: " << initial_conditions_.getNumLandmarks() << std::endl;
    
    std::cout << "\nIteration Data:" << std::endl;
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

std::vector<std::string> EKFSLAMDataLoader::parseCSVLine(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        tokens.push_back(token);
    }
    
    return tokens;
}

Matrix EKFSLAMDataLoader::reconstructMatrix(const std::vector<double>& data, int size) {
    Matrix mat(size, size);
    
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            mat(i, j) = data[i * size + j];
        }
    }
    
    return mat;
}