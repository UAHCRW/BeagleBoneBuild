#include "PressureSensor.hpp"
#include <bitset>
#include <iomanip>
#include <iostream>

// ---------------------------------------------------------------------------------------------------------------------
PressureSensor::PressureSensor(uint16_t i2cBus) : I2CDevice(i2cBus, PRESSURE_SENSOR_ADDR), configured_{false}
{
    // This sensor isnt very smart so no way to actually verify the sensor is good, that I know of.
    configured_ = isConnectionValid();
}

// ---------------------------------------------------------------------------------------------------------------------
float PressureSensor::getPressure()
{
    unsigned char dataBuffer[4];
    readRegisters(0x00, dataBuffer, 4);
    uint16_t val = 0;

    val |= (dataBuffer[0] & 0x3F) << 8;
    val |= dataBuffer[1];

    return (float)val * PRESSURE_SENSOR_SCALE_FACTOR;
}

// ---------------------------------------------------------------------------------------------------------------------
float PressureSensor::getTemperature()
{
    unsigned char dataBuffer[4];
    readRegisters(0x00, dataBuffer, 4);
    uint16_t val = 0;

    val |= dataBuffer[2] << 3;
    val |= dataBuffer[3] >> 5;

    return temperatureConversion(val);
}