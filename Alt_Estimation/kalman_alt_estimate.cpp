#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct LogData {
    double timestamp;
    double gps_altitude;
    double altimeter_1_altitude;
    double altimeter_2_altitude;
};

struct FilterConfig {
    const double process_noise_q = 0.01;
    const double max_altimeter_jump_m = 4.0;
    const double max_pair_difference_m = 2.0;
    const double ground_est_alpha = 1.0;

    const double transition_start_m = 15.0;
    const double transition_end_m = 30.0;
    const double low_altitude_r = 0.5;
    const double high_altitude_r = 5.0;
};

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

struct EstimateRow {
    double timestamp;
    double gps_altitude;
    double altimeter_1_altitude;
    double altimeter_2_altitude;
    double estimate_agl;
    double ground_est_msl;
    bool altimeter_1_valid;
    bool altimeter_2_valid;
};

std::vector<LogData> readCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open input file: " + filename);
    }

    std::vector<LogData> data;
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        LogData row{};
        char comma1 = '\0';
        char comma2 = '\0';
        char comma3 = '\0';

        if (!(ss >> row.timestamp >> comma1 >> row.gps_altitude >> comma2 >>
              row.altimeter_1_altitude >> comma3 >> row.altimeter_2_altitude) ||
            comma1 != ',' || comma2 != ',' || comma3 != ',') {
            continue;
        }

        data.push_back(row);
    }

    return data;
}

bool isValidAltimeterReading(double reading,
                             std::optional<double> previous_valid_reading,
                             double max_jump_m) {
    if (!std::isfinite(reading) || reading < 0.0) {
        return false;
    }

    if (!previous_valid_reading.has_value()) {
        return true;
    }

    return std::fabs(reading - *previous_valid_reading) <= max_jump_m;
}

bool areConsistentAltimeters(double alt1,
                             double alt2,
                             double max_pair_difference_m) {
    return std::isfinite(alt1) &&
           std::isfinite(alt2) &&
           alt1 >= 0.0 &&
           alt2 >= 0.0 &&
           std::fabs(alt1 - alt2) <= max_pair_difference_m;
}

double blendedMeasurementNoise(double predicted_agl, const FilterConfig& config) {
    const double span = std::max(config.transition_end_m - config.transition_start_m, 1e-6);
    const double weight = clamp01((predicted_agl - config.transition_start_m) / span);
    return (1.0 - weight) * config.low_altitude_r + weight * config.high_altitude_r;
}

bool isValidAltimeter(double reading,
                      std::optional<double> previous_valid_reading,
                      const FilterConfig& config) {
    if (!std::isfinite(reading) || reading < 0.0) {
        return false;
    }

    if (!previous_valid_reading.has_value()) {
        return true;
    }

    return std::fabs(reading - *previous_valid_reading) <= config.max_altimeter_jump_m;
}

std::vector<EstimateRow> runKalmanFilter(const std::vector<LogData>& log_data,
                                         const FilterConfig& config)
{
    if (log_data.empty()) {
        throw std::runtime_error("No valid rows were loaded.");
    }

    std::vector<EstimateRow> output;
    output.reserve(log_data.size());

    std::optional<double> previous_alt1;
    std::optional<double> previous_alt2;

    double xhat = 0.0;
    double P = 10.0;
    double ground_est_msl = 0.0;
    bool initialized = false;

    for (const LogData& sample : log_data) {

        bool alt1_valid = isValidAltimeter(sample.altimeter_1_altitude, previous_alt1, config);
        bool alt2_valid = isValidAltimeter(sample.altimeter_2_altitude, previous_alt2, config);
        const bool pair_consistent = areConsistentAltimeters(sample.altimeter_1_altitude, sample.altimeter_2_altitude, config.max_pair_difference_m);

        if (!alt1_valid && !alt2_valid && pair_consistent) {
            alt1_valid = true;
            alt2_valid = true;
        }

        if (!initialized) {
            if (alt1_valid && alt2_valid)
                xhat = (sample.altimeter_1_altitude + sample.altimeter_2_altitude) / 2.0;
            else if (alt1_valid)
                xhat = sample.altimeter_1_altitude;
            else if (alt2_valid)
                xhat = sample.altimeter_2_altitude;
            ground_est_msl = sample.gps_altitude - xhat;
            initialized = true;
        }

        const double gps_derived_agl = std::max(0.0, sample.gps_altitude - ground_est_msl);
        xhat = gps_derived_agl;

        // P: uncertainty in the current AGL estimate.
        // Q: uncertainty added during prediction before applying sensor corrections.
        // R: assumed altimeter noise, blended based on altitude regime.
        // K: correction weight between predicted AGL and measured AGL / Kalman Gain

        P += config.process_noise_q;

        if (alt1_valid) {
            
            double innovation1 = sample.altimeter_1_altitude - xhat;
            double R1 = blendedMeasurementNoise(xhat, config);

            double K1 = P / (P + R1);

            xhat = xhat + K1 * innovation1;
            P = (1.0 - K1) * P;
            previous_alt1 = sample.altimeter_1_altitude;
        }
        
        if (alt2_valid) {
                        
            double innovation2 = sample.altimeter_2_altitude - xhat;
            double R2 = blendedMeasurementNoise(xhat, config);

            double K2 = P / (P + R2);

            xhat = xhat + K2 * innovation2;
            P = (1.0 - K2) * P;
            previous_alt2 = sample.altimeter_2_altitude;
        }

        if (xhat < 0.0) {
            xhat = 0.0;
        }

        if (alt1_valid || alt2_valid) {
            ground_est_msl = (1.0 - config.ground_est_alpha) * ground_est_msl +
                config.ground_est_alpha * (sample.gps_altitude - xhat);
        }

        output.push_back(EstimateRow{
            sample.timestamp,
            sample.gps_altitude,
            sample.altimeter_1_altitude,
            sample.altimeter_2_altitude,
            xhat,
            ground_est_msl,
            alt1_valid,
            alt2_valid,
        });
    }

    return output;
}

void writeEstimates(const std::string& filename, const std::vector<EstimateRow>& rows) {
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        throw std::runtime_error("Unable to open output file: " + filename);
    }

    output_file << std::fixed << std::setprecision(6);
    for (const EstimateRow& row : rows) {
        output_file << row.timestamp << ' '
                    << row.gps_altitude << ' '
                    << row.altimeter_1_altitude << ' '
                    << row.altimeter_2_altitude << ' '
                    << row.estimate_agl << ' '
                    << row.ground_est_msl << ' '
                    << row.altimeter_1_valid << ' '
                    << row.altimeter_2_valid << '\n';
    }
}

void processLog(const std::string& input_csv, const std::string& output_txt) {
    const FilterConfig config;
    const std::vector<LogData> log_data = readCSV(input_csv);
    const std::vector<EstimateRow> rows = runKalmanFilter(log_data, config);
    writeEstimates(output_txt, rows);

    for (const EstimateRow& row : rows) {
        std::cout << "t=" << row.timestamp
                  << " gps=" << row.gps_altitude
                  << " alt1=" << row.altimeter_1_altitude
                  << " alt2=" << row.altimeter_2_altitude
                  << " est_agl=" << row.estimate_agl
                  << " ground_est=" << row.ground_est_msl
                  << " alt1_valid=" << row.altimeter_1_valid
                  << " alt2_valid=" << row.altimeter_2_valid
                  << '\n';
    }
}

int main() {
    try {
        processLog("log1.csv", "alt_estimate1.txt");
        processLog("log2.csv", "alt_estimate2.txt");
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    return 0;
}
