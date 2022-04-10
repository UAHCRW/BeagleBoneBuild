#include "main.hpp"
#include <exception>
#include <iomanip>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "measurement.hpp"

int main()
{
    initializePayload();
    printStartupConfig();

    if (isConfiguredForFlight())
    {
        // buzzer_.setValue(exploringBB::GPIO_VALUE::HIGH); // This is getting annoying real fast
        std::cout << "Buzzzzzzzz   " << std::flush;
        usleep(1e6);
        buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
        std::cout << "Buzzing off" << std::endl;
    }
    else
    {
        for (;;)
        {
            usleep(1e6);
            std::cout << "Buzzzzzzzz" << std::endl;
            buzzer_.setValue(exploringBB::GPIO_VALUE::HIGH);
            usleep(1e6 / 2);
            buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
        }
    }

    int flightDetectionCounter  = 0;
    int landingDetectionCounter = 0;
    bool flightDetected{false}, landingDetected{false};

    std::chrono::high_resolution_clock::time_point programStartTime_ = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point checkTime, lastSampleTime, sampleStart, sampleEnd;
    float mag;
    float motionThreshold = 10.5; // g's

    for (;;)
    {
        checkTime = std::chrono::high_resolution_clock::now();
        double msSinceLastSample = std::chrono::duration_cast<std::chrono::milliseconds>(checkTime - lastSampleTime).count();


        if (msSinceLastSample >= HEART_BEAT_PERIOD)
        {
            double timeStamp = 
                        std::chrono::duration_cast<std::chrono::milliseconds>(checkTime - programStartTime_).count();
            lastSampleTime = checkTime;

            Measurement meas;
            meas.staticPressure  = staticPressureSensor_.getPressure();
            meas.dynamicPressure = pitotProbe_.getPressure();
            accelerometer_.readState(meas.accelX, meas.accelY, meas.accelZ);
            teensy_.takeSample(meas.gyroX, meas.gyroY, meas.gyroZ);
            // std::cout << "Time: " << timeStamp << " => " << meas.toString();
            outFile_ << timeStamp << "," << meas.toCsv();
            outFile_.flush();

            mag = pow(pow(meas.accelX, 2) + pow(meas.accelY, 2) + pow(meas.accelZ, 2), 0.5);

            if (flightDetected)
            {
                landingDetectionCounter++;

                if (mag > motionThreshold)
                    landingDetectionCounter = 0;

                if (landingDetectionCounter > 1500 && !landingDetected)
                {
                    landingDetected = true;
                    std::string msg = "Landing Detected, through arduioius computation it has been determined that you" 
                                       "are in grid square A12. If not in grid square A12, please move there.\r\n";
                                      
                    std::cout << "Landing detected" << std::endl;
                    std::cout << msg;
                    xbee_.send(msg);
                }
            }
            else
            {
                if (mag > 10.5)
                    flightDetectionCounter++;

                if (flightDetectionCounter > 50)
                {
                    std::cout << "Take off detected" << std::endl;
                    flightDetected = true;
                }
            }
        }

        usleep(30);
    }

    return 1;
}

bool isConfiguredForFlight()
{
    return staticPressureSensor_.isConfigured() && pitotProbe_.isConfigured() && accelerometer_.isConfigured() &&
           xbee_.isConfigured() && outFile_.is_open();
}

void printStartupConfig()
{
    std::cout << "---------------------------------------------------------------------" << std::endl;
    std::cout << "          Dead Reckoning Navigation System (DRNS) Startup" << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;
    std::cout << "\tHeartbeat pin configured:   true" << std::endl;
    std::cout << "\tMagnetometer Configured:    " << std::boolalpha << magnetometer_.isConfigured() << std::endl;
    std::cout << "\tPressure Sensor Configured: " << std::boolalpha << staticPressureSensor_.isConfigured()
              << std::endl;
    std::cout << "\tPitot Probe Configured:     " << std::boolalpha << pitotProbe_.isConfigured() << std::endl;
    std::cout << "\tAccelerometer Configured:   " << std::boolalpha << accelerometer_.isConfigured() << std::endl;
    std::cout << "\tTeensy / Gyro Configured:   " << std::boolalpha << true << std::endl;
    std::cout << "\tXBee Configured:            " << std::boolalpha << xbee_.isConfigured() << std::endl;
    std::cout << "\tOutput File Open:           " << std::boolalpha << outFile_.is_open() << std::endl;
    std::cout << "\tSampling Frequency [Hz]:    " << HEART_BEAT_FREQUENCY << std::endl;
    std::cout << "\tSampling Period [ms]:       " << HEART_BEAT_PERIOD << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;
}

void initializePayload()
{
    outFile_.open("crw_payload_data.csv");
    heartbeatPin_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    heartbeatPin_.setValue(exploringBB::GPIO_VALUE::LOW);
    buzzer_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
}