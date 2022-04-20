#ifndef MEASUREMNT_HPP
#define MEASUREMNT_HPP

#include <string>
#include <Eigen/Dense>

struct Measurement
{
    float measurementTime;
    float staticPressure;
    float dynamicPressure;
    Eigen::Vector3f accel;
    Eigen::Vector3f gyro;
    Eigen::Vector3f mag;

    Measurement()
        : measurementTime{0.0}, 
          staticPressure{0.0},
          dynamicPressure{0.0}
    {
        for (int ii = 0; ii < 3; ii++)
        {
            accel[ii] = 0;
            gyro[ii] = 0;
            mag[ii] = 0;
        }
    }

    std::string toString()
    {
        std::stringstream ss;
        ss << "Time: "<< measurementTime << " [Static P.: " << staticPressure << "], [Dynamic P.: " << dynamicPressure
           << "], [Accel: " << accel[0] << ", " << accel[1] << ", " << accel[2] << "] [Gyro: " << gyro[0] << ", " << gyro[1]
           << ", " << gyro[2] << "] [Mag: " << mag[0] << ", " << mag[1] << ", " << mag[2] << "]";
        return ss.str();
    }

    std::string toXbee()
    {
        std::stringstream ss;
        ss <<"Time: "<< measurementTime << " [Static P.: " << staticPressure << "], [Dynamic P.: " << dynamicPressure
           << "], [Accel: " << accel[0] << ", " << accel[1] << ", " << accel[2] << "] [Gyro: " << gyro[0] << ", " << gyro[1]
           << ", " << gyro[2] << "] [Mag: " << mag[0] << ", " << mag[1] << ", " << mag[2] << "]\r\n"  << std::endl;
        return ss.str();
    }

    std::string toCsv()
    {
        std::stringstream ss;
        ss  << measurementTime << "," << staticPressure << "," << dynamicPressure
           << "," << accel[0] << "," << accel[1] << "," << accel[2] << "," << gyro[0] << "," << gyro[1]
           << "," << gyro[2] << "," << mag[0] << "," << mag[1] << "," << mag[2]  << std::endl;
        return ss.str();
    }
};

#endif