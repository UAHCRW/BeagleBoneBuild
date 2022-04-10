#include <string>

struct Measurement
{
    float staticPressure;
    float dynamicPressure;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;

    Measurement()
        : staticPressure{0.0},
          dynamicPressure{0.0}, accelX{0.0}, accelY{0.0}, accelZ{0.0}, gyroX{0.0}, gyroY{0.0}, gyroZ{0.0}
    {
    }

    std::string toString()
    {
        std::stringstream ss;
        ss << "Measurement: [Static Pressure: " << staticPressure << "], [Dynamic Pressure: " << dynamicPressure
           << "], [Accel: " << accelX << ", " << accelY << ", " << accelZ << "] [Gyro: " << gyroX << ", " << gyroY
           << ", " << gyroZ << "]" << std::endl;
        return ss.str();
    }

    std::string toXbee()
    {
        std::stringstream ss;
        ss << "Measurement: [Static Pressure: " << staticPressure << "], [Dynamic Pressure: " << dynamicPressure
           << "], [Accel: " << accelX << ", " << accelY << ", " << accelZ << "] [Gyro: " << gyroX << ", " << gyroY
           << ", " << gyroZ << "]\r\n";
        return ss.str();
    }

    std::string toCsv()
    {
        std::stringstream ss;
        ss << staticPressure << "," << dynamicPressure << "," << accelX << "," << accelY << "," << accelZ << ","
           << gyroX << "," << gyroY << "," << gyroZ << "\n";
        return ss.str();
    }
};