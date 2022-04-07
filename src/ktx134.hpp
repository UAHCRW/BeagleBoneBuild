#ifndef KTX134_HEADER
#define KTX134_HEADER
#include "I2CDevice.hpp"
#include "stdint.h"
#include <iostream>

#define KTX134_DEV_ADDR 0x1F

namespace KTX134
{
#define convRange8G  0.000244140751365703299
#define convRange16G 0.000488281197546311838
#define convRange32G 0.000976523950926236762
#define convRange64G 0.001953125095370342112

    enum class RegisterMap : uint8_t
    {
        MAN_ID        = 0x0,
        PART_ID       = 0x01,
        XADP_L        = 0x02,
        XADP_H        = 0x03,
        YADP_L        = 0x04,
        YADP_H        = 0x05,
        ZADP_L        = 0x06,
        ZADP_H        = 0x07,
        XOUT_L        = 0x08,
        XOUT_H        = 0x09,
        YOUT_L        = 0x0A,
        YOUT_H        = 0x0B,
        ZOUT_L        = 0x0C,
        ZOUT_H        = 0x0D,
        COTR          = 0x12,
        WHO_AM_I      = 0x13,
        TSCP          = 0x14,
        TSPP          = 0x15,
        INS1          = 0x16,
        INS2          = 0x17,
        INS3          = 0x18,
        STATUS_REG    = 0x19,
        INT_REL       = 0x1A,
        CNTL1         = 0x1B,
        CNTL2         = 0x1C,
        CNTL3         = 0x1D,
        CNTL4         = 0x1E,
        CNTL5         = 0x1F,
        CNTL6         = 0x20,
        ODCNTL        = 0x21,
        INC1          = 0x22,
        INC2          = 0x23,
        INC3          = 0x24,
        INC4          = 0x25,
        INC5          = 0x26,
        INC6          = 0x27,
        TILT_TIMER    = 0x29,
        TDTRC         = 0x2A,
        TDTC          = 0x2B,
        TTH           = 0x2C,
        TTL           = 0x2D,
        FTD           = 0x2E,
        STD           = 0x2F,
        TLT           = 0x30,
        TWS           = 0x31,
        FFTH          = 0x32,
        FFC           = 0x33,
        FFCNTL        = 0x34,
        TILT_ANGLE_LL = 0x37,
        TILT_ANGLE_HL = 0x38,
        HYST_SET      = 0x39,
        LP_CNTL1      = 0x3A,
        LP_CNTL2      = 0x3B,
        WUFTH         = 0x49,
        BTSWUFTH      = 0x4A,
        BTSTH         = 0x4B,
        BTSC          = 0x4C,
        WUFC          = 0x4D,
        SELF_TEST     = 0x5D,
        BUF_CNTL1     = 0x5E,
        BUF_CNTL2     = 0x5F,
        BUF_STATUS_1  = 0x60,
        BUF_STATUS_2  = 0x61,
        BUF_CLEAR     = 0x62,
        BUF_READ      = 0x63,
    };

    class KTX134 : public exploringBB::I2CDevice
    {
        public:
        KTX134(uint16_t i2cBus);
        ~KTX134(){};

        bool checkWhoAmI();

        bool isConfigured() { return configured_; }

        bool readState(float& x, float& y, float& z);

        private:
        void initialize();
        bool configured_;
    };
}
#endif