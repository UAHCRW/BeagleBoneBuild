#pragma once
#include "GPIO.hpp"
#include "LIS3MDL.hpp"
#include "PressureSensor.hpp"
#include "ktx134.hpp"
#include "teensy.hpp"
#include "xbee.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <vector>

#define HEART_BEAT_FREQUENCY  100 // Hz
#define HEART_BEAT_PIN_NUMBER 48

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pthread_t heartBeat_;
LIS3MDL magnetometer_(2);
PressureSensor staticPressureSensor_(2);
PressureSensor pitotProbe_(1);
KTX134::KTX134 accelerometer_(1);
Xbee xbee_;
Teensy teensy_;
std::vector<float> accelMags_;
std::ofstream outFile_;

exploringBB::GPIO buzzer_(60);
exploringBB::GPIO heartbeatPin_(HEART_BEAT_PIN_NUMBER);

bool getMeasurement = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Threaded functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Sends out pulse every specified time increment
void* payloadHeartBeat(void* arg)
{
    for (;;)
    {
        heartbeatPin_.setValue(exploringBB::GPIO_VALUE::HIGH);
        if (getMeasurement) std::cout << "Warning, missed an interrupt!" << std::endl;
        getMeasurement = true;
        usleep(1e6 / HEART_BEAT_FREQUENCY);
        heartbeatPin_.setValue(exploringBB::GPIO_VALUE::LOW);
        usleep(1e6 / HEART_BEAT_FREQUENCY);
    }
    return NULL;
}

float findMax(std::vector<float> data)
{
    float max = 0.0;
    for (auto n : data)
    {
        if (std::fabs(n) > max) max = n;
    }
    return max;
}

bool isConfiguredForFlight()
{
    return staticPressureSensor_.isConfigured() && pitotProbe_.isConfigured() && accelerometer_.isConfigured() &&
           xbee_.isConfigured() && outFile_.is_open();
}
