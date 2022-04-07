#ifndef LIS3MDL_HEADER
#define LIS3MDL_HEADER
#include "I2CDevice.hpp"
#include "stdint.h"
#include <iostream>

#define LIS3MDL_ADDR           0X1E
#define LIS3MDL_WHO_AM_I_VALUE 0X3D
#define LIS3MDL_4GAUSS_SCALE   6842
#define LIS3MDL_8GAUSS_SCALE   3421
#define LIS3MDL_12GAUSS_SCALE  2281
#define LIS3MDL_16GAUSS_SCALE  1711
#define LIS3MDL_TEMP_SCALE     8

enum class RegisterMap : uint16_t
{
    WHO_AM_I   = 0x0F,
    CTRL_REG1  = 0x20,
    CTRL_REG2  = 0x21,
    CTRL_REG3  = 0x22,
    CTRL_REG4  = 0x23,
    CTRL_REG5  = 0x24,
    STATUS_REG = 0x27,
    OUT_X_L    = 0x28,
    OUT_X_H    = 0x29,
    OUT_Y_L    = 0x2A,
    OUT_Y_H    = 0x2B,
    OUT_Z_L    = 0x2C,
    OUT_Z_H    = 0x2D,
    TEMP_OUT_L = 0x2E,
    TEMP_OUT_H = 0x2F,
    INT_CFG    = 0x30,
    INT_SRC    = 0x31,
    INT_THS_L  = 0x32,
    INT_THS_H  = 0x33,
};

enum class AxisOperatingMode : uint8_t
{
    LOW_POWER              = 0x00,
    MEDIUM_PERFORMANCE     = 0x01,
    HIGH_PERFORMANCE       = 0x02,
    ULTRA_HIGH_PERFORMANCE = 0x03,
};

enum class OdrConfiguration : uint8_t
{
    ODR_0_625 = 0x00,
    ODR_1_25  = 0x01,
    ODR_2_5   = 0x02,
    ODR_5     = 0x03,
    ODR_10    = 0x04,
    ODR_20    = 0x05,
    ODR_40    = 0x06,
    ODR_80    = 0x07,
};

enum class Range : uint8_t
{
    FOUR    = 0x00,
    EIGHT   = 0x01,
    TWELVE  = 0x02,
    SIXTEEN = 0x03,
};

enum class SpiInterfaceSelection : uint8_t
{
    FOUR_WIRE  = 0x00,
    THREE_WIRE = 0x01,
};

enum class OperatingMode : uint8_t
{
    CONTINIOUS        = 0x00,
    SINGLE_CONVERSION = 0x01,
    POWER_DOWN        = 0x02,
    POWER_UP          = 0x03,
};

enum class DataEndianess : uint8_t
{
    LSB_AT_LOWER = 0x00,
    MSB_AT_LOWER = 0x01,
};

struct MagMeasurement
{
    float x;
    float y;
    float z;
    float temp;

    MagMeasurement() : x{0.0}, y{0.0}, z{0.0}, temp{0.0} {};
};

class LIS3MDL : public exploringBB::I2CDevice
{
    public:
    LIS3MDL(uint16_t i2cBus);
    ~LIS3MDL(){};

    uint8_t getWhoAmI();
    bool checkWhoAmI() { return getWhoAmI() == LIS3MDL_WHO_AM_I_VALUE; }

    /// \brief Returns true when the sensor is configured and connection has been verified
    bool isConfigured() { return configured_; }

    /// \brief Gets a measurement from the magnetometer
    MagMeasurement getMeasurement();

    private:
    void setScaleFactor();
    void setCtrlReg1();
    void setCtrlReg2();
    void setCtrlReg3();
    void setCtrlReg4();
    int16_t readAxisMeasurementRaw(RegisterMap lowReg);

    bool configured_;
    uint32_t scaleFactor_;

    // CTRL_REG1
    bool tempEnabled_;
    AxisOperatingMode xyAxisMode_;
    OdrConfiguration odr_;
    bool fastOdr_;
    bool selfTestEnable_;

    // CTRL_REG2
    Range range_;
    bool reboot_;
    bool softReset_;

    // CTRL_REG3
    bool lowPowerMode_;
    SpiInterfaceSelection spiInterfaceSel_;
    OperatingMode operatingMode_;

    // CTRL_REG4
    AxisOperatingMode zAxisMode_;
    DataEndianess dataEndianessFmt_;
};
#endif