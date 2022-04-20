#pragma once
#include "GPIO.hpp"
#include "LIS2MDL.hpp"
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
#include <queue>
#include <mutex>
#include <pthread.h>


#define HEART_BEAT_FREQUENCY  100.0 // Hz
#define HEART_BEAT_PERIOD     1000.0 / HEART_BEAT_FREQUENCY
#define HEART_BEAT_PIN_NUMBER 48
#define X_START_POSITIONAL_OFFSET 1516.0
#define Y_START_POSITIONAL_OFFSET -1790.21

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor Objects
LIS2MDL magnetometer_(2);
PressureSensor staticPressureSensor_(2);
PressureSensor pitotProbe_(1);
KTX134::KTX134 accelerometer_(1);
Xbee xbee_;
Teensy teensy_;
exploringBB::GPIO buzzer_(60);
exploringBB::GPIO heartbeatPin_(HEART_BEAT_PIN_NUMBER);

// Output files
std::ofstream measurementFile_;
std::ofstream trajectoryFile_;

// Variables for calculating state updates
Eigen::VectorXf accelBias_;
Eigen::VectorXf gyroBias_;
std::queue<Measurement> measurementQue_;
std::mutex measurementQueMutex_;
pthread_t measurementProcessorThread_;
Measurement previousMeasurement_;
bool stopThread_ = false;
float finalPositionX, finalPositionY;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// \brief Returns True if all sensors are properly configured and output files have been created
bool isConfiguredForFlight();

/// \brief prints the startup message to the terminal
void printStartupConfig();

/// \brief Initializes in inputs / outputs that need configured during bootup
void initializePayload();

/// \brief Handles the measurements taken in the main thread
void *handleMeasurements(void *);

/// \brief Converts position to grid square location
std::string convertPositionToGridSquare();
