#include "xbee.hpp"

Xbee::Xbee() : file_(-1), isConfigured_(false)
{
    file_ = open("/dev/ttyO4", O_RDWR | O_NOCTTY | O_NDELAY);

    if (file_ < 0)
    {
        isConfigured_ = false;
        perror("UART: Failed to open device.\n");
    }

    struct termios options;
    tcgetattr(file_, &options);
    options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
    options.c_iflag = IGNPAR | ICRNL;
    tcflush(file_, TCIFLUSH);
    tcsetattr(file_, TCSANOW, &options);
    isConfigured_ = true;
}

void Xbee::send(const char* data, int size)
{
    if (write(file_, data, size) < 0) perror("UART: failed to write data");
}

void Xbee::send(std::string data)
{
    char buffer[data.length() + 1];
    strcpy(buffer, data.c_str());

    send(&buffer[0], sizeof(buffer));
}
