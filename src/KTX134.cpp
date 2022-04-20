#include "ktx134.hpp"
#include <bitset>
#include <iostream>

namespace KTX134
{
    KTX134::KTX134(uint16_t i2cBus) : I2CDevice(i2cBus, KTX134_DEV_ADDR), configured_{true}
    {
        configured_ &= checkWhoAmI();

        initialize();
    }

    bool KTX134::checkWhoAmI()
    {
        uint8_t value = 0;
        readRegister((uint16_t)RegisterMap::WHO_AM_I, value);
        return (value == 0x46);
    }

    void KTX134::initialize()
    {
        uint8_t val = 0;
        writeRegister((uint16_t)RegisterMap::CNTL1, 0);

        // Set the ODR in the CNTL4 Register
        val = 0;
        val |= 0x07;
        writeRegister((uint16_t)RegisterMap::ODCNTL, val);

        // Set the range in CNTL1 Register
        val = 0;
        val |= 0x03 << 6; // High performance modes
        val |= 0x02 << 3; // Sets range to 16G
        writeRegister((uint16_t)RegisterMap::CNTL1, val);
    }

    bool KTX134::readState(Measurement& meas)
    {
        uint8_t buff[6]{0};
        readRegisters((uint16_t)RegisterMap::XOUT_L, buff, 6);

        uint16_t tempVal = 0;
        tempVal |= buff[0];
        tempVal |= ((uint16_t)buff[1]) << 8;
        meas.accel[0] = (float)((int16_t)tempVal) * convRange32G * 9.80665;

        tempVal = 0;
        tempVal |= buff[2];
        tempVal |= ((uint16_t)buff[3]) << 8;
        meas.accel[1] = (float)((int16_t)tempVal) * convRange32G * 9.80665;

        tempVal = 0;
        tempVal |= buff[4];
        tempVal |= ((uint16_t)buff[5]) << 8;
        meas.accel[2] = (float)((int16_t)tempVal) * convRange32G * 9.80665;

        // Got to rotate the accelerometers frame to match that of the gyro
        meas.accel[0] = meas.accel[0] * -1.0;
        meas.accel[1] = meas.accel[1] * -1.0;

        return true;
    }

}
