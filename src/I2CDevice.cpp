/*
 * I2C.cpp  Created on: 17 May 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 * Made available for the book "Exploring BeagleBone"
 * See: www.exploringbeaglebone.com
 * Licensed under the EUPL V.1.1
 *
 * This Software is provided to You under the terms of the European
 * Union Public License (the "EUPL") version 1.1 as published by the
 * European Union. Any use of this Software, other than as authorized
 * under this License is strictly prohibited (to the extent such use
 * is covered by a right of the copyright holder of this Software).
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * For more details, see http://www.derekmolloy.ie/
 */
#include "I2CDevice.hpp"
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#define HEX(x) std::setw(2) << std::setfill('0') << "x" << std::hex << (uint16_t)(x)

namespace exploringBB
{
    I2CDevice::I2CDevice(unsigned int bus, unsigned int device) : connectionValid_{true}, devAddr_{device}
    {
        std::string name = BBB_I2C_2;
        if (bus == 0)
            name = BBB_I2C_0;
        else if (bus == 1)
            name = BBB_I2C_1;

        if ((file_ = open(name.c_str(), O_RDWR)) < 0)
        {
            perror("I2C: failed to open the bus\n");
            connectionValid_ = false;
        }

        if (ioctl(file_, I2C_SLAVE, devAddr_) < 0)
        {
            connectionValid_ = false;
            perror("I2C: Failed to connect to the device\n");
        }

        if (connectionValid_)
        {
            uint8_t val;
            connectionValid_ &= readRegister(0x00, val);
        }

        if (!connectionValid_)
        {
            std::cout << "I2C Device at " << HEX(devAddr_)
                      << " was not found. Check all connections. Run i2cdetect -y -r " << bus << std::endl;
        }
    }

    bool I2CDevice::writeRegister(unsigned int registerAddress, unsigned char value)
    {
        if (!connectionValid_) return false;

        unsigned char buffer[2];
        buffer[0] = registerAddress;
        buffer[1] = value;
        if (write(file_, buffer, 2) != 2)
        {
            std::cout << "I2C " << __func__ << " failed to write to device address " << HEX(devAddr_) << std::endl;
            return false;
        }
        return true;
    }

    bool I2CDevice::writeRegAddr(unsigned char value)
    {
        if (!connectionValid_) return false;

        unsigned char buffer[1];
        buffer[0] = value;
        if (write(file_, buffer, 1) != 1)
        {
            std::cout << "I2C " << __func__ << " failed to write to device address " << HEX(devAddr_) << std::endl;
            return false;
        }
        return true;
    }

    bool I2CDevice::readRegister(unsigned int registerAddress, uint8_t& value)
    {
        if (!connectionValid_)
        {
            value = 0;
            return false;
        }

        bool noFailure = true;
        noFailure &= writeRegAddr(registerAddress);
        unsigned char buffer[1];
        if (read(file_, buffer, 1) != 1)
        {
            std::cout << "I2C " << __func__ << " failed to read from device address " << HEX(devAddr_) << std::endl;
            return false;
        }
        value = buffer[0];
        return noFailure;
    }

    bool I2CDevice::readRegisters(unsigned int registerAddress, unsigned char* buffer, unsigned int bufferSize)
    {
        if (!connectionValid_)
        {
            for (uint16_t ii = 0; ii < bufferSize; ii++) { buffer[ii] = 0; }
            return false;
        }

        writeRegAddr(registerAddress);
        if (read(file_, buffer, bufferSize) != (int)bufferSize)
        {
            std::cout << "I2C " << __func__ << " failed to read from device address " << HEX(devAddr_) << std::endl;
            return false;
        }
        return true;
    }

    I2CDevice::~I2CDevice()
    {
        if (file_ != -1) close(file_);

        file_ = -1;
    }

} /* namespace exploringBB */
