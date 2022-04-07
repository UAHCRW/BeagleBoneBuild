#include "LIS3MDL.hpp"
#include "iostream"

// ---------------------------------------------------------------------------------------------------------------------
LIS3MDL::LIS3MDL(uint16_t i2cBus)
    : I2CDevice(i2cBus, LIS3MDL_ADDR), configured_{true}, scaleFactor_{0}, tempEnabled_{true},
      xyAxisMode_{AxisOperatingMode::ULTRA_HIGH_PERFORMANCE}, odr_{OdrConfiguration::ODR_80}, fastOdr_{false},
      selfTestEnable_{false}, range_{Range::TWELVE}, reboot_{false}, softReset_{false}, lowPowerMode_{false},
      spiInterfaceSel_{SpiInterfaceSelection::FOUR_WIRE}, operatingMode_{OperatingMode::CONTINIOUS},
      zAxisMode_{AxisOperatingMode::ULTRA_HIGH_PERFORMANCE}, dataEndianessFmt_{DataEndianess::LSB_AT_LOWER}
{
    configured_ &= isConnectionValid();
    configured_ &= checkWhoAmI();
    setScaleFactor();
    setCtrlReg1();
    setCtrlReg2();
    setCtrlReg3();
    setCtrlReg4();
}

// ---------------------------------------------------------------------------------------------------------------------
uint8_t LIS3MDL::getWhoAmI()
{
    uint8_t val;
    readRegister((uint8_t)RegisterMap::WHO_AM_I, val);
    return val;
}

// ---------------------------------------------------------------------------------------------------------------------
MagMeasurement LIS3MDL::getMeasurement()
{
    MagMeasurement meas;
    meas.x    = (float)readAxisMeasurementRaw(RegisterMap::OUT_X_L) / scaleFactor_;
    meas.y    = (float)readAxisMeasurementRaw(RegisterMap::OUT_Y_L) / scaleFactor_;
    meas.z    = (float)readAxisMeasurementRaw(RegisterMap::OUT_Z_L) / scaleFactor_;
    meas.temp = (float)readAxisMeasurementRaw(RegisterMap::TEMP_OUT_L) / LIS3MDL_TEMP_SCALE + 24;
    return meas;
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS3MDL::setScaleFactor()
{
    switch (range_)
    {
        case Range::FOUR: scaleFactor_ = LIS3MDL_4GAUSS_SCALE; break;
        case Range::EIGHT: scaleFactor_ = LIS3MDL_8GAUSS_SCALE; break;
        case Range::TWELVE: scaleFactor_ = LIS3MDL_12GAUSS_SCALE; break;
        case Range::SIXTEEN: scaleFactor_ = LIS3MDL_16GAUSS_SCALE; break;
        default: scaleFactor_ = 0; break;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS3MDL::setCtrlReg1()
{
    uint8_t val = 0;
    val |= (uint8_t)tempEnabled_ << 7;
    val |= (uint8_t)xyAxisMode_ << 5;
    val |= (uint8_t)odr_ << 3;
    val |= (uint8_t)fastOdr_ << 1;
    val |= (uint8_t)selfTestEnable_;
    selfTestEnable_ = false;
    writeRegister((uint16_t)RegisterMap::CTRL_REG1, val);
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS3MDL::setCtrlReg2()
{
    uint8_t val = 0;
    val |= (uint8_t)range_ << 5;
    val |= (uint8_t)reboot_ << 3;
    val |= (uint8_t)softReset_ << 2;
    reboot_    = false;
    softReset_ = false;
    writeRegister((uint16_t)RegisterMap::CTRL_REG2, val);
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS3MDL::setCtrlReg3()
{
    uint8_t val = 0;
    val |= (uint8_t)lowPowerMode_ << 5;
    val |= (uint8_t)spiInterfaceSel_ << 2;
    val |= (uint8_t)operatingMode_;
    writeRegister((uint16_t)RegisterMap::CTRL_REG3, val);
}

// ---------------------------------------------------------------------------------------------------------------------
void LIS3MDL::setCtrlReg4()
{
    uint8_t val = 0;
    val |= (uint8_t)zAxisMode_ << 2;
    val |= (uint8_t)dataEndianessFmt_ << 1;
    writeRegister((uint16_t)RegisterMap::CTRL_REG4, val);
}

// ---------------------------------------------------------------------------------------------------------------------
int16_t LIS3MDL::readAxisMeasurementRaw(RegisterMap lowReg)
{
    unsigned char buff[2];
    readRegisters((uint16_t)lowReg, buff, 2);
    uint16_t val = 0;
    val |= buff[1] << 8;
    val |= buff[0];
    return (int16_t)val;
}
