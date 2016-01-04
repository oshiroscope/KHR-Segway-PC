#include "serial_port.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

SerialPort::SerialPort(std::string serialPort)
    :m_port_name(serialPort.c_str()), m_baudrate(B115200)
{
    struct termios oldtio, newtio;
    m_fd = open(m_port_name, O_RDWR | O_NOCTTY);
    if(m_fd < 0){perror(m_port_name); exit(-1);}
    
    tcgetattr(m_fd, &oldtio);
    bzero(&newtio, sizeof(newtio));

    cfsetspeed(&newtio, m_baudrate);

    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag = CS8;
    newtio.c_cflag |= (CLOCAL | CREAD);

    newtio.c_cflag &= ~CSTOPB;

    newtio.c_cflag |= PARENB; 
    newtio.c_cflag &= ~PARODD; 
    newtio.c_cflag &= ~CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 5;

    tcflush(m_fd, TCIFLUSH);
    tcsetattr(m_fd, TCSANOW, &newtio);
}

int SerialPort::Read(unsigned char *rx, int size)
{
    return read(m_fd, rx, size);
}

int SerialPort::Write(unsigned char *tx, int size)
{
    return write(m_fd, tx, size);
}
