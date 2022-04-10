#pragma once
#include "GPIO.hpp"
#include "LIS3MDL.hpp"
#include "PressureSensor.hpp"
#include "ktx134.hpp"
#include "teensy.hpp"
#include "xbee.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <Eigen/Dense>
#include <ctime>
 
using Eigen::MatrixXd;

#define HEART_BEAT_FREQUENCY  100.0 // Hz
#define HEART_BEAT_PERIOD     1000.0 / HEART_BEAT_FREQUENCY
#define HEART_BEAT_PIN_NUMBER 48

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor Objects
LIS3MDL magnetometer_(2);
PressureSensor staticPressureSensor_(2);
PressureSensor pitotProbe_(1);
KTX134::KTX134 accelerometer_(1);
Xbee xbee_;
Teensy teensy_;
exploringBB::GPIO buzzer_(60);
exploringBB::GPIO heartbeatPin_(HEART_BEAT_PIN_NUMBER);

// Output files
std::ofstream outFile_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief Returns True if all sensors are properly configured and output files have been created
bool isConfiguredForFlight();

/// \brief prints the startup message to the terminal
void printStartupConfig();

/// \brief Initializes in inputs / outputs that need configured during bootup
void initializePayload();
