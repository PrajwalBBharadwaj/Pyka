#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

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
    std::vector<LogData> data;
    std::string line;
    std::getline(file, line);                   // Skip first line of CS file
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        LogData log;
        char comma;
        ss >> log.timestamp >> comma >> log.gps_altitude >> comma >> log.altimeter_1_altitude >> comma >> log.altimeter_2_altitude;
        data.push_back(log);
    }
    return data;
}

// Function for sensor fusion using Kalman filter
std::vector<double> sensorFusion(const std::vector<double>& gps, const std::vector<double>& alt1, const std::vector<double>& alt2, double& P, double& I)
{
    std::vector<double> estimate;
    double R, Q;

    // Define initial offset between GPS and Altimeters
    double offset = gps[0] - (alt1[0] + alt2[0]) / 2;

    // Loop through all values of GPS and Altimeters
    for (size_t i = 0; i < gps.size(); ++i)
    {
        // Set Q,R based on distance to ground (Need smoothness above 15m, accuracy below 15 m)
        if (gps[i] - offset > 15)
        {
            R = 500;
            Q = 0.01;
        }
        else
        {
            R = 0.5;
            Q = 0.0001;
        }
        // Prediction 1
        double xhat = gps[i] - offset;
        P = P + Q;

        // Update 1
        double G = P / (P + R);
        P = (I - G) * P;
        xhat = xhat + G * (alt1[i] - xhat);

        // Prediction 2
        P = P + Q;

        // Update 2
        G = P / (P + R);
        P = (I - G) * P;
        xhat = xhat + G * (alt2[i] - xhat);
        offset = gps[i] - xhat;
        estimate.push_back(xhat);

        std::cout << "GPS : " << gps[i] << "| Alt1 : " << alt1[i] << "| Alt2 : " << alt2[i] << "| Estimate : " << estimate[i] << std::endl;
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
