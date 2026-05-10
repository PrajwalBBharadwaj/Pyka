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
    double low_altitude_threshold_m = 15.0;
    double process_noise_q = 0.05;
    double low_altitude_measurement_r = 0.8;
    double high_altitude_measurement_r = 8.0;
    double max_altimeter_jump_m = 4.0;
    double max_sensor_disagreement_m = 5.0;
    double max_innovation_m = 10.0;
};

struct EstimateRow {
    double timestamp;
    double gps_altitude;
    double altimeter_1_altitude;
    double altimeter_2_altitude;
    double estimate_agl;
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

std::optional<double> combineValidAltimeters(double alt1,
                                             bool alt1_valid,
                                             double alt2,
                                             bool alt2_valid,
                                             const FilterConfig& config) {
    if (alt1_valid && alt2_valid) {
        if (std::fabs(alt1 - alt2) <= config.max_sensor_disagreement_m) {
            return 0.5 * (alt1 + alt2);
        }

        return std::min(alt1, alt2);
    }

    if (alt1_valid) {
        return alt1;
    }

    if (alt2_valid) {
        return alt2;
    }

    return std::nullopt;
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
    bool initialized = false;

    for (const LogData& sample : log_data) {
        bool alt1_valid = isValidAltimeter(sample.altimeter_1_altitude, previous_alt1, config);
        bool alt2_valid = isValidAltimeter(sample.altimeter_2_altitude, previous_alt2, config);

        if (alt1_valid) {
            previous_alt1 = sample.altimeter_1_altitude;
        }
        if (alt2_valid) {
            previous_alt2 = sample.altimeter_2_altitude;
        }

        const std::optional<double> measurement = combineValidAltimeters(
            sample.altimeter_1_altitude,
            alt1_valid,
            sample.altimeter_2_altitude,
            alt2_valid,
            config);

        if (!initialized) {
            xhat = measurement.value_or(0.0);
            initialized = true;
        }

        P += config.process_noise_q;

        if (measurement.has_value()) {
            double predicted_height = xhat;
            double innovation = *measurement - predicted_height;

            double gated_measurement, R;

            if (std::fabs(innovation) > config.max_innovation_m) {
                gated_measurement = predicted_height + std::copysign(config.max_innovation_m, innovation);
            } else {
                gated_measurement = *measurement;
            }
                
            if (predicted_height <= config.low_altitude_threshold_m) {
                R = config.low_altitude_measurement_r;
            } else {
                R = config.high_altitude_measurement_r;
            }

            double K = P / (P + R);

            xhat = predicted_height + K * (gated_measurement - predicted_height);
            P = (1.0 - K) * P;
        }

        if (xhat < 0.0) {
            xhat = 0.0;
        }

        output.push_back(EstimateRow{
            sample.timestamp,
            sample.gps_altitude,
            sample.altimeter_1_altitude,
            sample.altimeter_2_altitude,
            xhat,
            alt1_valid,
            alt2_valid,
        });
    }

    return output;
}

void writeEstimates(const std::string& filename, const std::vector<EstimateRow>& rows)
{
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
                    << row.altimeter_1_valid << ' '
                    << row.altimeter_2_valid << '\n';
    }
}

void processLog(const std::string& input_csv, const std::string& output_txt)
{
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
                  << " alt1_valid=" << row.altimeter_1_valid
                  << " alt2_valid=" << row.altimeter_2_valid
                  << '\n';
    }
}

int main()
{
    try {
        processLog("log1.csv", "alt_estimate1.txt");
        processLog("log2.csv", "alt_estimate2.txt");
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    return 0;
}
