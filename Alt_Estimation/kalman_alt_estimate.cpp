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
    double low_altitude_threshold_m = 15.0;
    double transition_band_m = 4.0;
    double process_noise_q = 0.01;
    double low_altitude_measurement_r = 0.8;
    double high_altitude_measurement_r = 500.0;
    double gps_fallback_blend = 0.25;
    double ground_est_alpha = 0.5;
    double max_altimeter_jump_m = 4.0;
    double max_sensor_disagreement_m = 5.0;
    double max_innovation_m = 50.0;
};

double clamp01(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

double transitionWeight(double estimate_agl, const FilterConfig& config)
{
    const double band = std::max(config.transition_band_m, 1e-6);
    const double start = config.low_altitude_threshold_m - 0.5 * band;
    return clamp01((estimate_agl - start) / band);
}

double blendedMeasurementNoise(double estimate_agl, const FilterConfig& config)
{
    const double w = transitionWeight(estimate_agl, config);
    return (1.0 - w) * config.low_altitude_measurement_r +
           w * config.high_altitude_measurement_r;
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

bool shouldUseSensor(double sensor_reading,
                     double other_sensor_reading,
                     bool other_sensor_valid,
                     double estimate_agl,
                     const FilterConfig& config)
{
    if (!other_sensor_valid) {
        return true;
    }

    if (std::fabs(sensor_reading - other_sensor_reading) <= config.max_sensor_disagreement_m) {
        return true;
    }

    if (estimate_agl <= config.low_altitude_threshold_m) {
        return sensor_reading <= other_sensor_reading;
    }

    return std::fabs(sensor_reading - estimate_agl) <=
           std::fabs(other_sensor_reading - estimate_agl);
}

bool applyKalmanMeasurement(double measurement,
                            double gps_altitude,
                            double& xhat,
                            double& P,
                            double& ground_est_msl,
                            const FilterConfig& config)
{
    const double predicted_height = xhat;
    const double innovation = measurement - predicted_height;

    double gated_measurement = measurement;
    if (std::fabs(innovation) > config.max_innovation_m) {
        gated_measurement =
            predicted_height + std::copysign(config.max_innovation_m, innovation);
    }

    const double R = blendedMeasurementNoise(predicted_height, config);
    const double K = P / (P + R);

    xhat = predicted_height + K * (gated_measurement - predicted_height);
    P = (1.0 - K) * P;
    ground_est_msl = (1.0 - config.ground_est_alpha) * ground_est_msl +
                     config.ground_est_alpha * (gps_altitude - gated_measurement);
    return true;
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

        if (alt1_valid) {
            previous_alt1 = sample.altimeter_1_altitude;
        }
        if (alt2_valid) {
            previous_alt2 = sample.altimeter_2_altitude;
        }

        const bool use_alt1 = alt1_valid && shouldUseSensor(
            sample.altimeter_1_altitude,
            sample.altimeter_2_altitude,
            alt2_valid,
            xhat,
            config);
        const bool use_alt2 = alt2_valid && shouldUseSensor(
            sample.altimeter_2_altitude,
            sample.altimeter_1_altitude,
            alt1_valid,
            xhat,
            config);

        if (!initialized) {
            if (use_alt1 && use_alt2) {
                xhat = 0.5 * (sample.altimeter_1_altitude + sample.altimeter_2_altitude);
            } else if (use_alt1) {
                xhat = sample.altimeter_1_altitude;
            } else if (use_alt2) {
                xhat = sample.altimeter_2_altitude;
            } else {
                xhat = 0.0;
            }
            ground_est_msl = sample.gps_altitude - xhat;
            initialized = true;
        }

        const double gps_fallback_agl = std::max(0.0, sample.gps_altitude - ground_est_msl);
        xhat = (1.0 - config.gps_fallback_blend) * xhat + config.gps_fallback_blend * gps_fallback_agl;

        P += config.process_noise_q;

        bool applied_measurement = false;
        if (use_alt1) {
            applied_measurement = applyKalmanMeasurement(
                sample.altimeter_1_altitude,
                sample.gps_altitude,
                xhat,
                P,
                ground_est_msl,
                config);
        }
        if (use_alt2) {
            applied_measurement = applyKalmanMeasurement(
                sample.altimeter_2_altitude,
                sample.gps_altitude,
                xhat,
                P,
                ground_est_msl,
                config) || applied_measurement;
        }

        if (!applied_measurement) {
            xhat = gps_fallback_agl;
            P = std::min(P + config.process_noise_q, 50.0);
        } else {
            P = std::max(P, 1e-6);
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
            ground_est_msl,
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
                    << row.ground_est_msl << ' '
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
                  << " ground_est=" << row.ground_est_msl
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
