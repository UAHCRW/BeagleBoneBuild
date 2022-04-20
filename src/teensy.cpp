#include "teensy.hpp"
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
using std::ofstream;

Teensy::Teensy() : file_(-1), isConfigured_(false)
{
    file_ = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY);

    if (file_ < 0)
    {
        isConfigured_ = false;
        perror("UART: Failed to open device.\n");
    }

    struct termios options;
    tcgetattr(file_, &options);
    options.c_cflag = (B115200 | CS8 | CREAD | CLOCAL);
    options.c_iflag = IGNPAR | ICRNL;
    tcflush(file_, TCIOFLUSH);
    tcsetattr(file_, TCSANOW, &options);
}

void Teensy::send(char* data, int size)
{
    // std::cout << "Data is getting sent to teensy ";
    // for (int ii = 0; ii < size; ii++)
    //     std::cout << data[ii];
    // std::cout << std::endl;
    if (write(file_, data, size) < 0) perror("UART: failed to write data");
}

void Teensy::send(std::string data)
{
    // std::cout << "Sending " << data << " to teensy." << std::endl;
    char buffer[data.length() + 1];
    strcpy(buffer, data.c_str());

    send(&buffer[0], sizeof(buffer));
}

void Teensy::flushBuffer()
{
    // usleep(2e6);
    // tcflush(file_, TCIOFLUSH);

    // TODO: Shouldn't have to do this at all. This clears the data out of what ever buffer its stuck in
    // Teensy will receive a measurement when this happens instead of nothing
    send("\n");
}

void Teensy::receive(float& x, float& y, float& z)
{
    char newByte;
    char data[255];
    int ii             = 0;
    int timeoutCounter = 0;

    while (true)
    {
        timeoutCounter++;
        if (read(file_, &newByte, 1) > 0)
        {
            if (newByte == '\n' && ii > 0)
            {
                ii = 0;
                std::vector<std::string> v;
                std::stringstream ss(data);

                while (ss.good())
                {
                    std::string substr;
                    getline(ss, substr, ',');
                    v.push_back(substr);
                }
                if (v.size() == 3)
                {
                    try
                    {
                        x = std::stof(v[0]);
                        y = std::stof(v[1]);
                        z = std::stof(v[2]);
                    }
                    catch (...)
                    {
                        std::cout << "Warning couldnt convert all of the gyro values to numbers!" << std::endl;
                    }
                }
                else
                    std::cout << "Warning! data received that didn't haves 3 parameters only had " << v.size() <<
                                 ". Returned 0. " << ss.str() << std::endl;
                flushBuffer();
                return;
            }
            else
            {
                timeoutCounter = 0;
                data[ii++] = newByte;
            }
        }

        if (timeoutCounter > 1e6)
        {
            std::cout << "Teensy timed out on taking a sample!!!!!" << std::endl;
            flushBuffer();
            break;
        }
    }
}

void Teensy::takeSample(Measurement& meas)
{
    float x, y, z;
    receive(x, y, z);
    meas.gyro[0] = x;
    meas.gyro[1] = y;
    meas.gyro[2] = z;

    meas.gyro = meas.gyro * M_PI / 180.0;
}

void Teensy::requestSample()
{
    send("!\n");
}

bool Teensy::checkConfiguration()
{
    send("#\n");
    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();

    for (;;)
    {
        double passedSeconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - startTime).count();

        if (passedSeconds > 30)
        {
            std::cout << "Teensy never responded to call. " << std::endl;
            isConfigured_ = false;
            flushBuffer();
            return false;
        }

        char newByte;
        if (read(file_, &newByte, 1) > 0)
        {
            if (newByte == '#')
            {
                flushBuffer();
                isConfigured_ = true;
                return true;
            }
        }
    }
    
}
