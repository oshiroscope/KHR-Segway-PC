#include <iostream>
#include <map>
#include <vector>
#include <unistd.h>

#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"

int main()
{
    SerialPort sensor_port(SENSOR_SERIAL_PORT);
    SerialPort khr_port(KHR_SERIAL_PORT);    
    unsigned char read_buf[255];

    std::map <int, int> dest;
    dest[6] = 8000;
    dest[7] = 7000;
    dest[8] = 4700;
    dest[9] = 10300;
    auto cmd = CommandGen::SeriesServoMove(dest, 20);
    khr_port.Write(&cmd[0], cmd.size());
    sleep(1);
    
    // std::map <int, int> dest2;
    // dest2[2] = 32767;
    // dest2[3] = 32767;
    // dest2[6] = 7500;
    // dest2[7] = 7500;
    // dest2[8] = 32767;
    // dest2[9] = 32767;
    // auto cmd2 = CommandGen::SeriesServoMove(dest2, 20);
    // khr_port.Write(&cmd2[0], cmd2.size());
    // usleep(100000);

    dest.clear();
    dest[6] = 7500;
    dest[7] = 7500;
    cmd.clear();
    cmd = CommandGen::SeriesServoMove(dest, 20);
    khr_port.Write(&cmd[0], cmd.size());
    usleep(100000);

    while(1){
	sensor_port.Read(read_buf, 7);
	float theta = *((float *)read_buf);
	std::cout << theta << std::endl;
	
	std::map <int, int> dest;
	dest[18] = (float)7500 + theta * (float)1000;
	dest[19] = (float)7500 - theta * (float)1000;
	auto cmd = CommandGen::SeriesServoMove(dest, 5);
	khr_port.Write(&cmd[0], cmd.size());
    }
}

