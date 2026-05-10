#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <optional>
#include <cmath>

// Define a struct to hold log data
struct LogData {
    double timestamp;
    double gps_altitude;
    double altimeter_1_altitude;
    double altimeter_2_altitude;
};

// Function to read log data from CSV file
std::vector<LogData> readCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Unable to open input file: " + filename);
    }
    std::vector<LogData> data;
    std::string line;
    std::getline(file, line);                   // Skip first line of CS file
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        LogData log{};
        char comma1 = '\0';
        char comma2 = '\0';
        char comma3 = '\0';
        if (!(ss >> log.timestamp >> comma1 >> log.gps_altitude >> comma2 >>
              log.altimeter_1_altitude >> comma3 >> log.altimeter_2_altitude) ||
            comma1 != ',' || comma2 != ',' || comma3 != ',')
        {
            continue;
        }
        data.push_back(log);
    }
    return data;
}

bool isValidAltimeterReading(double reading,
                             std::optional<double> previous_valid_reading,
                             double max_jump_m)
{
    if (!std::isfinite(reading) || reading < 0.0)
    {
        return false;
    }

    if (!previous_valid_reading.has_value())
    {
        return true;
    }

    return std::fabs(reading - *previous_valid_reading) <= max_jump_m;
}

bool areConsistentAltimeters(double alt1,
                             double alt2,
                             double max_pair_difference_m)
{
    return std::isfinite(alt1) &&
           std::isfinite(alt2) &&
           alt1 >= 0.0 &&
           alt2 >= 0.0 &&
           std::fabs(alt1 - alt2) <= max_pair_difference_m;
}

double clamp01(double value)
{
    if (value < 0.0)
    {
        return 0.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}

double blendedMeasurementNoise(double predicted_agl,
                               double transition_start_m,
                               double transition_end_m,
                               double low_altitude_r,
                               double high_altitude_r)
{
    const double span = std::max(transition_end_m - transition_start_m, 1e-6);
    const double weight = clamp01((predicted_agl - transition_start_m) / span);
    return (1.0 - weight) * low_altitude_r + weight * high_altitude_r;
}

// Function for sensor fusion using Kalman filter
std::vector<double> sensorFusion(const std::vector<double>& gps, const std::vector<double>& alt1, const std::vector<double>& alt2, double& P, double& I)
{
    std::vector<double> estimate;
    double R;
    const double Q = 0.1;
    const double transition_start_m = 15.0;
    const double transition_end_m = 30.0;
    const double low_altitude_r = 0.5;
    const double high_altitude_r = 50.0;
    const double max_altimeter_jump_m = 4.0;
    const double max_pair_difference_m = 2.0;
    std::optional<double> previous_valid_alt1;
    std::optional<double> previous_valid_alt2;

    // Estimate ground elevation relative to mean sea level.
    double ground_msl = gps[0];
    // Loop through all values of GPS and Altimeters
    for (size_t i = 0; i < gps.size(); ++i)
    {
        bool alt1_valid = isValidAltimeterReading(alt1[i], previous_valid_alt1, max_altimeter_jump_m);
        bool alt2_valid = isValidAltimeterReading(alt2[i], previous_valid_alt2, max_altimeter_jump_m);
        const bool pair_consistent = areConsistentAltimeters(alt1[i], alt2[i], max_pair_difference_m);

        if (!alt1_valid && !alt2_valid && pair_consistent)
        {
            alt1_valid = true;
            alt2_valid = true;
        }

        if (alt1_valid)
        {
            previous_valid_alt1 = alt1[i];
        }
        if (alt2_valid)
        {
            previous_valid_alt2 = alt2[i];
        }

        // Blend measurement trust smoothly from 15 m to 30 m.
        const double predicted_agl = gps[i] - ground_msl;
        R = blendedMeasurementNoise(
            predicted_agl,
            transition_start_m,
            transition_end_m,
            low_altitude_r,
            high_altitude_r);
        // Prediction 1
        double xhat = predicted_agl;
        P = P + Q;

        // Update 1
        if (alt1_valid)
        {
            double G = P / (P + R);
            P = (I - G) * P;
            xhat = xhat + G * (alt1[i] - xhat);
        }

        // Prediction 2
        P = P + Q;

        // Update 2
        if (alt2_valid)
        {
            double G = P / (P + R);
            P = (I - G) * P;
            xhat = xhat + G * (alt2[i] - xhat);
        }

        if (xhat < 0.0)
        {
            xhat = 0.0;
        }

        ground_msl = gps[i] - xhat;
        estimate.push_back(xhat);

        std::cout << "GPS : " << gps[i]
                  << "| Alt1 : " << alt1[i]
                  << "| Alt2 : " << alt2[i]
                  << "| Estimate : " << estimate[i]
                  << "| Alt1 valid : " << alt1_valid
                  << "| Alt2 valid : " << alt2_valid
                  << std::endl;
    }
    return estimate;
}

void runLog1Kalman()
{
    // Read log data
    std::vector<LogData> logData = readCSV("log1.csv");

    // Extract data from log
    std::vector<double> time, gps, alt1, alt2;
    for (const auto& data : logData)
    {
        time.push_back(data.timestamp);
        gps.push_back(data.gps_altitude);
        alt1.push_back(data.altimeter_1_altitude);
        alt2.push_back(data.altimeter_2_altitude);
    }

    // Initialize variables for Kalman filter
    double P = 1, I = 1;

    // Perform sensor fusion using Kalman filter
    std::vector<double> altEstimate = sensorFusion(gps, alt1, alt2, P, I);

    // Write sensor fusion data to a file
    std::ofstream outputFile("alt_estimate1.txt");
    if (outputFile.is_open())
    {
        for (size_t i = 0; i < altEstimate.size(); ++i) {
            outputFile << time[i] << " " << gps[i] << " " << alt1[i] << " " << alt2[i] << " " << altEstimate[i] << "\n";
        }
        outputFile.close();
    }
    else
    {
        std::cerr << "Unable to open output file";
    }
}

void runLog2Kalman()
{
    // Read log data
    std::vector<LogData> logData = readCSV("log2.csv");

    // Extract data from log
    std::vector<double> time, gps, alt1, alt2;
    for (const auto& data : logData)
    {
        time.push_back(data.timestamp);
        gps.push_back(data.gps_altitude);
        alt1.push_back(data.altimeter_1_altitude);
        alt2.push_back(data.altimeter_2_altitude);
    }

    // Initialize variables for Kalman filter
    double P = 1, I = 1;

    // Perform sensor fusion using Kalman filter
    std::vector<double> altEstimate = sensorFusion(gps, alt1, alt2, P, I);

    // Write sensor fusion data to a file
    std::ofstream outputFile("alt_estimate2.txt");
    if (outputFile.is_open())
    {
        for (size_t i = 0; i < altEstimate.size(); ++i) {
            outputFile << time[i] << " " << gps[i] << " " << alt1[i] << " " << alt2[i] << " " << altEstimate[i] << "\n";
        }
        outputFile.close();
    }
    else
    {
        std::cerr << "Unable to open output file";
    }
}




int main()
{
    runLog1Kalman();
    runLog2Kalman();
    return 0;
}
