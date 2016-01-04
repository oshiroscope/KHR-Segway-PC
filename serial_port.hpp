#pragma once
#include <string>

class SerialPort{
public:
    SerialPort(std::string serialPort);

    int Read(unsigned char *rx, int size);
    int Write(unsigned char *tx, int size);
    
private:
    int m_fd;
    const char *m_port_name;
    const int m_baudrate;
};
