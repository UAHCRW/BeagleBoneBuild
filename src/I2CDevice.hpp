/*
 * I2C.h  Created on: 17 May 2014
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

#ifndef I2C_H_
#define I2C_H_

#define BBB_I2C_0 "/dev/i2c-0"
#define BBB_I2C_1 "/dev/i2c-1"
#define BBB_I2C_2 "/dev/i2c-2"
#include <stdint.h>

namespace exploringBB
{
    class I2CDevice
    {

        public:
        I2CDevice(unsigned int bus, unsigned int device);
        ~I2CDevice();
        bool writeRegAddr(unsigned char value);
        bool writeRegister(unsigned int registerAddress, unsigned char value);
        bool writeRegisters(unsigned int registerAddress, unsigned char* buffer, unsigned int buffSize);
        bool readRegister(unsigned int registerAddress, uint8_t& value);
        bool readRegisters(unsigned int registerAddress, unsigned char* buffer, unsigned int bufferSize);
        bool isConnectionValid() { return connectionValid_; }

        private:
        int file_;
        bool connectionValid_;
        unsigned int devAddr_;
    };

} /* namespace exploringBB */

#endif /* I2C_H_ */
