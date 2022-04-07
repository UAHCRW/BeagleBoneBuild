#include "main.hpp"
#include <exception>
#include <iomanip>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <string.h>

int main()
{
    outFile_.open("crw_payload_data.csv");
    std::cout << "---------------------------------------------------------------------" << std::endl;
    std::cout << "          Dead Reckoning Navigation System (DRNS) Startup" << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;
    heartbeatPin_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    heartbeatPin_.setValue(exploringBB::GPIO_VALUE::LOW);
    buzzer_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
    std::cout << "\tHeartbeat pin configured:   true" << std::endl;
    std::cout << "\tMagnetometer Configured:    " << std::boolalpha << magnetometer_.isConfigured() << std::endl;
    std::cout << "\tPressure Sensor Configured: " << std::boolalpha << staticPressureSensor_.isConfigured()
              << std::endl;
    std::cout << "\tPitot Probe Configured:     " << std::boolalpha << pitotProbe_.isConfigured() << std::endl;
    std::cout << "\tAccelerometer Configured:   " << std::boolalpha << accelerometer_.isConfigured() << std::endl;
    std::cout << "\tTeensy / Gyro Configured:   " << std::boolalpha << true << std::endl;
    std::cout << "\tXBee Configured:            " << std::boolalpha << xbee_.isConfigured() << std::endl;
    std::cout << "\tOutput File Open:           " << std::boolalpha << outFile_.is_open() << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;

    if (isConfiguredForFlight())
    {
        usleep(2e6);
        buzzer_.setValue(exploringBB::GPIO_VALUE::HIGH);
        usleep(1e6);
        buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
    }
    else
    {
        for (;;)
        {
            usleep(1e6);
            buzzer_.setValue(exploringBB::GPIO_VALUE::HIGH);
            usleep(1e6 / 2);
            buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
        }
    }

    std::cout << "Heart has been started" << std::endl;
    pthread_create(&heartBeat_, NULL, &payloadHeartBeat, NULL);

    int flightDetectionCounter  = 0;
    int landingDetectionCounter = 0;
    bool flightDetected{false}, landingDetected{false};

    for (;;)
    {
        float mag;

        if (getMeasurement)
        {
            getMeasurement = false;
            Measurement meas;
            meas.staticPressure  = staticPressureSensor_.getPressure();
            meas.dynamicPressure = pitotProbe_.getPressure();
            accelerometer_.readState(meas.accelX, meas.accelY, meas.accelZ);
            teensy_.receive(meas.gyroX, meas.gyroY, meas.gyroZ);
            // std::cout << meas.toString();
            outFile_ << meas.toCsv();
            outFile_.flush();

            mag = pow(pow(meas.accelX, 2) + pow(meas.accelY, 2) + pow(meas.accelZ, 2), 0.5);

            if (mag > 10.5)
            {
                if (flightDetectionCounter++ >= 50 && !flightDetected)
                {
                    std::cout << "Take off detected!" << std::endl;
                    flightDetected = true;
                }
                // else if (!flightDetected && flightDetectionCounter < 50)
                //     std::cout << "Flight Detected " << flightDetectionCounter << std::endl;
                else if (flightDetected)
                {
                    landingDetectionCounter = 0;
                    // std::cout << "Mag " << mag << std::endl;
                }
            }

            if (mag < 10.5 && flightDetected)
            {
                landingDetectionCounter++;
                // std::cout << "Accel below 10.5 " << mag << std::endl;

                if (landingDetectionCounter > 300 && !landingDetected)
                {
                    landingDetected = true;
                    std::cout << "Landing detected" << std::endl;
                    xbee_.send("Landing Detected, through arduioius computation it has been determined that you are in "
                               "grid square A12. If not in grid square A12, please move there.\r\n");
                }
            }
        }

        usleep(30);
    }

    return 1;
}