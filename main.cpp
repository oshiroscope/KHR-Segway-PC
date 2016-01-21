#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>

#include <opencv2/opencv.hpp>
#include <aruco.h>

#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"
#include "key_input.hpp"
#include "motion.hpp"
#include "odometry.hpp"

char key;

std::vector<char> key_buf;

int main()
{
    Odometry odometry;
    odometry.Start();

    KeyInput keyInput;
    keyInput.Start();
    //set_term();

    //SerialPort sensor_port(SENSOR_SERIAL_PORT);
    SerialPort khr_port(KHR_SERIAL_PORT);

    Motion motion(khr_port);
    std::map<int, int> dest;
    motion.Init(dest);
    motion.Move(dest);
    sleep(1);

    motion.Clear(dest);
    motion.Grub(dest);
    motion.Move(dest);
    sleep(1);

    char c, old_c, state;
    unsigned char read_buf[255];
    
    float theta, omega;

    while(1){
	/*sensor_port.Read(read_buf, 9);
	unsigned char sum = 0;
	for(int i = 0; i < 8; i++){
	    sum += read_buf[i];
	}

	if(sum != read_buf[8])
	    std::cout << "checksum error" << std::endl;
	else{
	    theta = *((float *)read_buf);
	    omega = *((float *)(read_buf + 4));
	}*/
	
	std::cout << std::fixed << std::setprecision(10) << theta << "\t"
	  << std::fixed << std::setprecision(10) << omega << "\t";

	motion.Clear(dest);

	switch(keyInput.GetKey())
	{
	case 'w':
	    std::cout << "forward";
	    motion.Forward(dest);
	    motion.Move(dest, 20);
	    break;
	case 'a':
	    std::cout << "left";
	    motion.Left(dest);
	    motion.Move(dest, 20);
	    break;
	case 's':
	    std::cout << "backward";
	    motion.Backward(dest);
	    motion.Move(dest, 20);
	    break;
	case 'd':
	    std::cout << "right";
	    motion.Right(dest);
	    motion.Move(dest, 20);
	    break;
	default :
	    std::cout << "none";
	    motion.None(dest);
	    //motion.Stab(dest, theta, omega);
	    motion.Move(dest, 10);
	    break;
	}


	//std::cout << "\t" << key;
	std::cout << std::endl;
	usleep(10000);	
    }
}

