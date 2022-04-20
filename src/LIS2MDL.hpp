#ifndef LIS2MDL_HEADER
#define LIS2MDL_HEADER
#include "I2CDevice.hpp"
#include "stdint.h"
#include <iostream>
#include <Eigen/Dense>
#include "measurement.hpp"

#define LIS2MDL_ADDR           0X1E
#define LIS2MDL_WHO_AM_I_VALUE 0X40
#define LIS2MDL_SCALE_FACTOR   1.5
#define LIS2MDL_TEMP_SCALE     8

enum class Lis2RegisterMap : uint16_t
{
    WHO_AM_I   = 0x4F,
    CFG_REG_A  = 0x60,
    OUTX_L_REG = 0x68,
    OUTX_H_REG = 0x69,
    OUTY_L_REG = 0x6A,
    OUTY_H_REG = 0x6B,
    OUTZ_L_REG = 0x6C,
    OUTZ_H_REG = 0x6D,
};

class LIS2MDL : public exploringBB::I2CDevice
{
    public:
    LIS2MDL(uint16_t i2cBus);
    ~LIS2MDL(){};

    uint8_t getWhoAmI();
    bool checkWhoAmI() { return getWhoAmI() == LIS2MDL_WHO_AM_I_VALUE; }

    /// \brief Returns true when the sensor is configured and connection has been verified
    bool isConfigured() { return configured_; }

    /// \brief Gets a measurement from the magnetometer
    void getMeasurement(Measurement& meas);

    void calibrateSensor();

    void printCalibrationMatrices();

    private:
    void setConfigRegA();

    bool configured_;

    Eigen::MatrixXf a_matrix_;
    Eigen::MatrixXf b_matrix_;
};
#endif