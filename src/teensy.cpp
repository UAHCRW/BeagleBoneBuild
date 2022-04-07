#include "teensy.hpp"
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
    options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
    options.c_iflag = IGNPAR | ICRNL;
    tcflush(file_, TCIFLUSH);
    tcsetattr(file_, TCSANOW, &options);
}

void Teensy::send(char* data, int size)
{
    if (write(file_, data, size) < 0) perror("UART: failed to write data");
}

void Teensy::receive(float& x, float& y, float& z)
{
    char newByte;
    char data[255];
    int ii             = 0;
    bool noMeasurement = true;

    while (noMeasurement)
    {
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
                    std::cout << "Warning! data received that didn't haves 3 parameters. Returned 0." << std::endl;
                // std::cout << "Parsed data: " << x << ", " << y << ", " << z << std::endl;
                noMeasurement = false;
                return;
            }
            else
                data[ii++] = newByte;
        }
    }
}
