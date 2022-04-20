#include "LIS2MDL.hpp"
#include "iostream"
#include <Eigen/Core>
// ---------------------------------------------------------------------------------------------------------------------
LIS2MDL::LIS2MDL(uint16_t i2cBus)
    : I2CDevice(i2cBus, LIS2MDL_ADDR), configured_{true}
{
    configured_ &= isConnectionValid();
    configured_ &= checkWhoAmI();

    if (configured_)
        setConfigRegA();

    // Configure the calibration matrices
    a_matrix_.resize(3,3);
    a_matrix_(0,0) = 2.17231459;
    a_matrix_(0,1) = -0.04305537;
    a_matrix_(0,2) =  -0.02645279;
    a_matrix_(1,0) = -0.04305537;
    a_matrix_(1,1) =   2.2034971;
    a_matrix_(1,2) =    0.02974335;
    a_matrix_(2,0) = -0.02645279;
    a_matrix_(2,1) =   0.02974335;
    a_matrix_(2,2) =   2.25164522;

    b_matrix_.resize(3,1);
    b_matrix_(0,0) = 0.11439748;
    b_matrix_(1,0) = 0.03041577;  
    b_matrix_(2,0) = 0.03175254;
}

// ---------------------------------------------------------------------------------------------------------------------
uint8_t LIS2MDL::getWhoAmI()
{
    uint8_t val;
    readRegister((uint8_t)Lis2RegisterMap::WHO_AM_I, val);
    return val;
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS2MDL::getMeasurement(Measurement& meas)
{
    uint8_t data[6];
    readRegisters((uint8_t)Lis2RegisterMap::OUTX_L_REG, &data[0], 6);

    Eigen::VectorXf rawMeas(3);


    uint16_t val = 0;
    val |= data[0];
    val |= (uint16_t)data[1] << 8;
    rawMeas(0) = (float)((int16_t)val) * LIS2MDL_SCALE_FACTOR / 1000;
    
    val = 0;
    val |= data[2];
    val |= (uint16_t)data[3] << 8;
    rawMeas(1) = (float)((int16_t)val) * LIS2MDL_SCALE_FACTOR / 1000;

    val = 0;
    val |= data[4];
    val |= (uint16_t)data[5] << 8;
    rawMeas(2) = (float)((int16_t)val) * LIS2MDL_SCALE_FACTOR / 1000;

    rawMeas = rawMeas - b_matrix_; // Correct for the hard iron offset

    meas.mag[0] = a_matrix_.row(0).dot(rawMeas);
    meas.mag[1] = a_matrix_.row(1).dot(rawMeas);
    meas.mag[2] = a_matrix_.row(2).dot(rawMeas);
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS2MDL::setConfigRegA()
{
    uint8_t val = 0x0C;
    writeRegister((uint8_t)Lis2RegisterMap::CFG_REG_A, val);
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS2MDL::calibrateSensor()
{
    std::cout << "Starting Magnetometer Calibration" << std::endl;
    Eigen::MatrixXd sampleData(2000, 3); // 2000 Samples which should equate to 20 seconds worth of data collecting

    

}

// ---------------------------------------------------------------------------------------------------------------------
void LIS2MDL::printCalibrationMatrices()
{
    std::cout << "A Matrix: " << std::endl;
    std::cout << a_matrix_ << std::endl << std::endl;

    std::cout << "B Matrix: " << std::endl;
    std::cout << b_matrix_ << std::endl << std::endl;
}