#ifndef PRESSURE_SENSOR_HPP
#define PRESSURE_SENSOR_HPP

#include "I2CDevice.hpp"
#include "stdint.h"

#define PRESSURE_SENSOR_ADDR         0X28
#define PRESSURE_SENSOR_SCALE_FACTOR 30 / 16393

class PressureSensor : protected exploringBB::I2CDevice
{
    public:
    PressureSensor(uint16_t i2cBus);
    ~PressureSensor(){};

    bool isConfigured() { return configured_; }

    float getPressure();
    float getTemperature();

    private:
    bool configured_;

    float temperatureConversion(uint16_t val) { return (float)val * 105 / 2047 - 20; }
};
#endif