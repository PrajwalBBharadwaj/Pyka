#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
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
    double nominal_dt_s = 0.01;
    double max_valid_height_m = 150.0;
    double max_altimeter_jump_m = 4.0;
    double max_sensor_disagreement_m = 5.0;
    double max_innovation_m = 8.0;
    double max_prediction_disagreement_m = 12.0;
    double process_velocity_damping = 0.92;
    double process_noise_position = 0.25;
    double process_noise_velocity = 4.0;
    double gps_ground_alpha = 0.04;
    double low_alt_gain = 0.55;
    double high_alt_gain = 0.18;
    double low_altimeter_trust = 0.75;
    double high_altimeter_trust = 0.35;
};

struct SensorSample {
    double timestamp;
    double gps_altitude;
    double altimeter_1_altitude;
    double altimeter_2_altitude;
    double estimate_agl;
    double estimate_ground_msl;
    bool altimeter_1_valid;
    bool altimeter_2_valid;
};

struct AltitudeState {
    double height_agl = 0.0;
    double vertical_velocity = 0.0;
    double variance_position = 25.0;
    double variance_velocity = 4.0;
    double estimated_ground_msl = 0.0;
    bool initialized = false;
};

std::vector<LogData> readCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open input file: " + filename);
    }

    std::vector<LogData> data;
    std::string line;
    std::getline(file, line);  // Skip header.

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        LogData log{};
        char comma1 = '\0';
        char comma2 = '\0';
        char comma3 = '\0';

        if (!(ss >> log.timestamp >> comma1 >> log.gps_altitude >> comma2 >>
              log.altimeter_1_altitude >> comma3 >> log.altimeter_2_altitude) ||
            comma1 != ',' || comma2 != ',' || comma3 != ',') {
            continue;
        }

        data.push_back(log);
    }

    return data;
}

bool isPlausibleAltimeterReading(double reading,
                                 std::optional<double> previous_reading,
                                 double max_valid_height_m,
                                 double max_jump_m)
{
    if (!std::isfinite(reading) || reading < 0.0 || reading > max_valid_height_m) {
        return false;
    }

    if (!previous_reading.has_value()) {
        return true;
    }

    return std::fabs(reading - *previous_reading) <= max_jump_m;
}

bool isConsistentWithPrediction(double reading,
                                double predicted_height,
                                double max_prediction_disagreement_m,
                                bool state_initialized)
{
    if (!state_initialized) {
        return true;
    }

    return std::fabs(reading - predicted_height) <= max_prediction_disagreement_m;
}

std::optional<double> combineAltimeters(double alt1,
                                        bool alt1_valid,
                                        double alt2,
                                        bool alt2_valid,
                                        double max_sensor_disagreement_m)
{
    if (alt1_valid && alt2_valid) {
        if (std::fabs(alt1 - alt2) <= max_sensor_disagreement_m) {
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

double clampMeasurementToInnovationGate(double predicted_height,
                                        double measured_height,
                                        double max_innovation_m)
{
    const double innovation = measured_height - predicted_height;
    if (std::fabs(innovation) <= max_innovation_m) {
        return measured_height;
    }

    return predicted_height + std::copysign(max_innovation_m, innovation);
}

void initializeState(AltitudeState& state,
                     const LogData& sample,
                     const std::optional<double>& combined_altimeter)
{
    const double initial_height = combined_altimeter.value_or(0.0);
    state.height_agl = initial_height;
    state.vertical_velocity = 0.0;
    state.estimated_ground_msl = sample.gps_altitude - initial_height;
    state.initialized = true;
}

double computeDt(const LogData& current, const LogData& previous, double nominal_dt_s)
{
    const double raw_dt = current.timestamp - previous.timestamp;
    if (!std::isfinite(raw_dt) || raw_dt <= 0.0) {
        return nominal_dt_s;
    }

    return raw_dt * nominal_dt_s;
}

SensorSample updateState(const LogData& sample,
                         const LogData* previous_sample,
                         std::optional<double>& previous_alt1,
                         std::optional<double>& previous_alt2,
                         AltitudeState& state,
                         const FilterConfig& config)
{
    const double dt = previous_sample == nullptr
        ? config.nominal_dt_s
        : computeDt(sample, *previous_sample, config.nominal_dt_s);

    const double bootstrap_height = state.initialized ? state.height_agl : 0.0;
    const double bootstrap_velocity = state.initialized ? state.vertical_velocity : 0.0;
    const double predicted_height = std::max(0.0, bootstrap_height + bootstrap_velocity * dt);
    const double predicted_velocity = state.vertical_velocity * config.process_velocity_damping;

    bool alt1_valid = isPlausibleAltimeterReading(
        sample.altimeter_1_altitude,
        previous_alt1,
        config.max_valid_height_m,
        config.max_altimeter_jump_m);
    alt1_valid = alt1_valid && isConsistentWithPrediction(
        sample.altimeter_1_altitude,
        predicted_height,
        config.max_prediction_disagreement_m,
        state.initialized);

    bool alt2_valid = isPlausibleAltimeterReading(
        sample.altimeter_2_altitude,
        previous_alt2,
        config.max_valid_height_m,
        config.max_altimeter_jump_m);
    alt2_valid = alt2_valid && isConsistentWithPrediction(
        sample.altimeter_2_altitude,
        predicted_height,
        config.max_prediction_disagreement_m,
        state.initialized);

    if (alt1_valid) {
        previous_alt1 = sample.altimeter_1_altitude;
    }
    if (alt2_valid) {
        previous_alt2 = sample.altimeter_2_altitude;
    }

    const std::optional<double> combined_altimeter = combineAltimeters(
        sample.altimeter_1_altitude,
        alt1_valid,
        sample.altimeter_2_altitude,
        alt2_valid,
        config.max_sensor_disagreement_m);

    if (!state.initialized) {
        initializeState(state, sample, combined_altimeter);
    }

    state.variance_position += config.process_noise_position * dt;
    state.variance_velocity += config.process_noise_velocity * dt;

    double fused_height = predicted_height;
    bool used_altimeter = false;

    if (combined_altimeter.has_value()) {
        const double gated_measurement = clampMeasurementToInnovationGate(
            predicted_height,
            *combined_altimeter,
            config.max_innovation_m);
        const bool low_altitude_mode = predicted_height <= config.low_altitude_threshold_m;
        const double base_gain = low_altitude_mode ? config.low_alt_gain : config.high_alt_gain;
        const double variance_weight = state.variance_position / (state.variance_position + 1.0);
        const double trust_scale = low_altitude_mode ? config.low_altimeter_trust : config.high_altimeter_trust;
        const double gain = std::clamp(base_gain * trust_scale * variance_weight, 0.05, 0.85);

        fused_height = predicted_height + gain * (gated_measurement - predicted_height);
        state.vertical_velocity = predicted_velocity + (fused_height - predicted_height) / std::max(dt, 1e-6);
        state.variance_position *= (1.0 - gain);

        const double ground_from_measurement = sample.gps_altitude - fused_height;
        state.estimated_ground_msl +=
            config.gps_ground_alpha * (ground_from_measurement - state.estimated_ground_msl);
        used_altimeter = true;
    } else {
        const double gps_fallback_height = std::max(0.0, sample.gps_altitude - state.estimated_ground_msl);
        fused_height = 0.85 * predicted_height + 0.15 * gps_fallback_height;
        state.vertical_velocity = (fused_height - state.height_agl) / std::max(dt, 1e-6);
        state.variance_position = std::min(state.variance_position + 0.5, 50.0);
    }

    state.height_agl = std::max(0.0, fused_height);

    if (!used_altimeter) {
        state.estimated_ground_msl = sample.gps_altitude - state.height_agl;
    }

    return SensorSample{
        sample.timestamp,
        sample.gps_altitude,
        sample.altimeter_1_altitude,
        sample.altimeter_2_altitude,
        state.height_agl,
        state.estimated_ground_msl,
        alt1_valid,
        alt2_valid,
    };
}

std::vector<SensorSample> estimateAltitude(const std::vector<LogData>& log_data,
                                           const FilterConfig& config)
{
    std::vector<SensorSample> estimates;
    estimates.reserve(log_data.size());

    AltitudeState state;
    std::optional<double> previous_alt1;
    std::optional<double> previous_alt2;

    for (size_t i = 0; i < log_data.size(); ++i) {
        const LogData* previous_sample = i == 0 ? nullptr : &log_data[i - 1];
        estimates.push_back(updateState(
            log_data[i],
            previous_sample,
            previous_alt1,
            previous_alt2,
            state,
            config));
    }

    return estimates;
}

void writeEstimates(const std::string& filename, const std::vector<SensorSample>& estimates)
{
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        throw std::runtime_error("Unable to open output file: " + filename);
    }

    output_file << std::fixed << std::setprecision(6);
    for (const SensorSample& sample : estimates) {
        output_file << sample.timestamp << ' '
                    << sample.gps_altitude << ' '
                    << sample.altimeter_1_altitude << ' '
                    << sample.altimeter_2_altitude << ' '
                    << sample.estimate_agl << ' '
                    << sample.estimate_ground_msl << ' '
                    << sample.altimeter_1_valid << ' '
                    << sample.altimeter_2_valid << '\n';
    }
}

void processLog(const std::string& input_csv, const std::string& output_txt)
{
    const std::vector<LogData> log_data = readCSV(input_csv);
    if (log_data.empty()) {
        throw std::runtime_error("No valid rows found in input file: " + input_csv);
    }

    const FilterConfig config;
    const std::vector<SensorSample> estimates = estimateAltitude(log_data, config);
    writeEstimates(output_txt, estimates);

    for (const SensorSample& sample : estimates) {
        std::cout << "t=" << sample.timestamp
                  << " gps=" << sample.gps_altitude
                  << " alt1=" << sample.altimeter_1_altitude
                  << " alt2=" << sample.altimeter_2_altitude
                  << " est_agl=" << sample.estimate_agl
                  << " alt1_valid=" << sample.altimeter_1_valid
                  << " alt2_valid=" << sample.altimeter_2_valid
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
