#ifndef XBEE_HPP
#define XBEE_HPP

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <termios.h>
#include <unistd.h>
using std::ofstream;

class Xbee
{
    public:
    Xbee();
    ~Xbee() { close(file_); }

    void send(const char* data, int size);
    void send(std::string data);
    bool isConfigured() { return isConfigured_; }

    private:
    int file_;
    bool isConfigured_;
};
#endif