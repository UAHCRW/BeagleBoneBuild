#ifndef TEENSY_HPP
#define TEENSY_HPP

#include <fcntl.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <termios.h>
#include <unistd.h>
using std::ofstream;

class Teensy
{
    public:
    Teensy();
    ~Teensy() { close(file_); }

    void send(char* data, int size);
    void send(std::string data);
    void receive(float& x, float& y, float& z);

    void takeSample(float&x, float&y, float& z);

    void flushBuffer();

    private:
    int file_;
    bool isConfigured_;
};
#endif