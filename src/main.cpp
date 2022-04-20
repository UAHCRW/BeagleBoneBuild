#include "main.hpp"
#include <exception>
#include <iomanip>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "measurement.hpp"
#include <map>

int main()
{
    initializePayload();
    printStartupConfig();

    if (isConfiguredForFlight())
    {
        usleep(2e6);
        buzzer_.setValue(exploringBB::GPIO_VALUE::HIGH); // This is getting annoying real fast
        std::cout << "Buzzzzzzzz   " << std::flush;
        usleep(1e6);
        buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);
        std::cout << "Buzzing off" << std::endl;
        usleep(2e6);
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

    Measurement meas;
    accelerometer_.readState(meas);
    magnetometer_.getMeasurement(meas);
    float roll = atan2(meas.accel[0], meas.accel[1]) * 180 / M_PI;
    float pitch = atan2(meas.accel[2], meas.accel[1]) * 180 / M_PI;
    float yaw = -atan2(meas.mag[0], meas.mag[2]) * 180 / M_PI;
    std::cout << "Initial Payload state on launch rail is => Roll: " << roll << "  Pitch: " << pitch << "  Yaw: " << yaw << std::endl;

    int flightDetectionCounter  = 0;
    int landingDetectionCounter = 0;
    bool flightDetected{false}, landingDetected{false};

    std::chrono::high_resolution_clock::time_point programStartTime_ = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point checkTime, lastSampleTime, sampleStart, sampleEnd;
    float mag;
    float motionThreshold = 10.5; // g's
    uint64_t ii = 0;
    double sampleTimeAccum = 0.0;

    pthread_create(&measurementProcessorThread_, NULL, handleMeasurements, NULL);

    for (;;)
    {
        checkTime = std::chrono::high_resolution_clock::now();
        double msSinceLastSample = std::chrono::duration_cast<std::chrono::milliseconds>(checkTime - lastSampleTime).count();


        if (msSinceLastSample >= HEART_BEAT_PERIOD)
        {
            Measurement meas;
            sampleStart = std::chrono::high_resolution_clock::now();
            meas.measurementTime = std::chrono::duration_cast<std::chrono::milliseconds>(checkTime - programStartTime_).count();
            lastSampleTime = checkTime;
            teensy_.requestSample();

            meas.staticPressure  = staticPressureSensor_.getPressure();
            meas.dynamicPressure = pitotProbe_.getPressure();
            accelerometer_.readState(meas);
            // magnetometer_.getMeasurement(meas);
            teensy_.takeSample(meas);
            
            try
            {
            measurementFile_ << meas.toCsv();
            measurementFile_.flush();
            }
            catch (const std::exception& e)
            {
                std::cout << "Error! " << e.what() << std::endl;
            }
        

            
            mag = meas.accel.norm();

            if (ii++ % 100 == 0)
                std::cout << meas.toString() << " " << mag << std::endl;


            if (mag > 9.9 || flightDetected)
            {
                measurementQueMutex_.lock();
                measurementQue_.push(meas);
                measurementQueMutex_.unlock();
            }


            if (flightDetected)
            {
                landingDetectionCounter++;

                if (mag > motionThreshold)
                    landingDetectionCounter = 0;

                if (landingDetectionCounter > 1500 && !landingDetected)
                {
                    landingDetected = true;
                    stopThread_ = true;
                    pthread_join(measurementProcessorThread_, NULL);
                    
                    std::string msg = "UAH CRW Payload is in " + convertPositionToGridSquare() + "\r\n";
                                      
                    std::cout << msg;
                    xbee_.send(msg);
                    break;
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
            sampleEnd = std::chrono::high_resolution_clock::now();
            sampleTimeAccum += 
                        std::chrono::duration_cast<std::chrono::milliseconds>(sampleEnd - sampleStart).count();
        }

        if (ii % 100 == 0 && sampleTimeAccum > 0.1)
        {
            // std::cout << "Avg sample time is " << sampleTimeAccum / 100.0 << std::endl;
            sampleTimeAccum = 0.0;
        }
    }

    return 1;
}

bool isConfiguredForFlight()
{
    return staticPressureSensor_.isConfigured() && pitotProbe_.isConfigured() && accelerometer_.isConfigured() &&
           xbee_.isConfigured() && measurementFile_.is_open() && teensy_.isConfigured() && magnetometer_.isConfigured()
           && trajectoryFile_.is_open();
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
    std::cout << "\tTeensy / Gyro Configured:   " << std::boolalpha << teensy_.isConfigured() << std::endl;
    std::cout << "\tXBee Configured:            " << std::boolalpha << xbee_.isConfigured() << std::endl;
    std::cout << "\tMeasurement File Open:      " << std::boolalpha << measurementFile_.is_open() << std::endl;
    std::cout << "\tTrajectory File Open:       " << std::boolalpha << trajectoryFile_.is_open() << std::endl;
    std::cout << "\tSampling Frequency [Hz]:    " << HEART_BEAT_FREQUENCY << std::endl;
    std::cout << "\tSampling Period [ms]:       " << HEART_BEAT_PERIOD << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;
    std::cout << "\nCalibration Matrices for the Magnetometer are:\n";
    magnetometer_.printCalibrationMatrices(); 
}

void initializePayload()
{
    // Initialize sensors and files
    measurementFile_.open("/media/ssd/crw_payload_data.csv");
    trajectoryFile_.open("/media/ssd/crw_trajectory_file.csv");
    heartbeatPin_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    heartbeatPin_.setValue(exploringBB::GPIO_VALUE::LOW);
    buzzer_.setDirection(exploringBB::GPIO_DIRECTION::OUTPUT);
    buzzer_.setValue(exploringBB::GPIO_VALUE::LOW);

    accelBias_.resize(3);
    gyroBias_.resize(3);

    accelBias_.setZero();
    gyroBias_.setZero();

    teensy_.checkConfiguration();
}

void *handleMeasurements(void *)
{
    float gravityOffset = -9.81;

    Measurement newMeasurement_;
    Eigen::Vector3f unBiasedAccel;
    Eigen::Vector3f unBiasedGyro;
    Eigen::Vector3f angularChange;

    Eigen::Matrix3f rotationMatrix = Eigen::MatrixXf::Identity(3,3);
    Eigen::Matrix3f oldRotationMatrix = Eigen::MatrixXf::Identity(3,3);

    Eigen::Matrix3f updateMatrix = Eigen::MatrixXf::Identity(3,3);
    Eigen::Matrix3f b_mat = Eigen::MatrixXf::Identity(3,3);
    Eigen::Matrix3f identityMatrix = Eigen::MatrixXf::Identity(3,3);

    Eigen::Vector3f rotatedAccelerations;
    
    Eigen::Vector3f velocity;
    Eigen::Vector3f prevVelocity;
    Eigen::Vector3f position;
    Eigen::Vector3f prevPos;
    velocity.setZero();
    prevVelocity.setZero();
    prevPos.setZero();
    position.setZero();

    int ii = 0;


    for (;;)
    {
        if (stopThread_ && measurementQue_.empty())
            break;

        if (!measurementQue_.empty())
        {
            // Get the new measurement but don't keep the mutex for very long
            measurementQueMutex_.lock();
            newMeasurement_ = measurementQue_.front();
            measurementQue_.pop();
            measurementQueMutex_.unlock();

            if (std::isnan(newMeasurement_.accel[0]))
                std::cout << "Accel is nan!" << std::endl;

            if (std::isnan(newMeasurement_.gyro[0]))
                std::cout << "Gyro is nan!" << std::endl;
            

            // Process the new measurement
            /////////////////////////////////
            unBiasedAccel = newMeasurement_.accel - accelBias_;
            unBiasedGyro = newMeasurement_.gyro - gyroBias_;
            float dt = (newMeasurement_.measurementTime - previousMeasurement_.measurementTime) / 1000.0;

            if (dt > 0.012)
            {
                previousMeasurement_ = newMeasurement_;
                continue;
            }

            // Calculate the rotation matrix
            angularChange = unBiasedGyro * dt;
            float sigma = angularChange.norm();

            b_mat(0,0) = 0.0;
            b_mat(0,1) = angularChange[2];
            b_mat(0,2) = -angularChange[1];
            b_mat(1,0) = -angularChange[2];
            b_mat(1,1) = 0.0;
            b_mat(1,2) = angularChange[0];
            b_mat(2,0) = angularChange[1];
            b_mat(2,1) = -angularChange[0];
            b_mat(2,2) = 0.0;

            updateMatrix = identityMatrix + (sin(sigma) / sigma) * b_mat + ((1 - cos(sigma)) / sigma / sigma) * b_mat * b_mat;
            rotationMatrix = oldRotationMatrix * updateMatrix;
            oldRotationMatrix = rotationMatrix;

            rotatedAccelerations = rotationMatrix * unBiasedAccel;
            rotatedAccelerations[1] = rotatedAccelerations[1] + gravityOffset;

            velocity = prevVelocity + rotatedAccelerations * dt;
            prevVelocity = velocity;

            position = prevPos + velocity * dt;
            prevPos = position;
            try
            {
                trajectoryFile_ << newMeasurement_.measurementTime << "," << position[0] << "," << position[1] << "," << position[2] << "," 
                            << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," 
                            << rotatedAccelerations[0] << "," << rotatedAccelerations[1] << "," 
                            << rotatedAccelerations[2] << std::endl;
                trajectoryFile_.flush();
            }
            catch (const std::exception& e)
            {
                std::cout << "Error! " << e.what() << std::endl;
            }

            finalPositionX = position[0];
            finalPositionY = position[2];
            previousMeasurement_ = newMeasurement_;

            if (std::isnan(prevPos[0]))
                std::cout << "Pos is nan!" << std::endl;

            if (ii++ % 100 == 0)
            {
                std::cout << "Pos [ " << finalPositionX << ", " << finalPositionY << ", " << position[2] 
                          << "]  Vel [ " << velocity[0] << ", " << velocity[1] << ", " << velocity[2]  << " ]"<< std::endl;
            }
        }
        else
            usleep(1000); // One millisecond
    }
    return NULL;
}

std::string convertPositionToGridSquare()
{
    int numCols = finalPositionX / 250.0;
    int numRows = finalPositionY / 250.0;

    finalPositionX = finalPositionX - X_START_POSITIONAL_OFFSET;
    finalPositionY = finalPositionY - Y_START_POSITIONAL_OFFSET;

    

    std::string colMap[7]{"A", "B", "C", "D", "E", "F", "G"};
    std::string letter = std::to_string(numCols);

    if (numCols >= 0 && numCols <= 6)
        letter = colMap[numCols];
    

    std::stringstream grid;
    grid << "Grid [" << numRows << ", " << letter << "]" << std::endl;
    std::cout << "Final X: " << finalPositionX << "  Final Y: " << finalPositionY << std::endl;
    return grid.str();
}