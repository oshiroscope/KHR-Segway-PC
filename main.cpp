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

int main()
{
    Odometry odometry;
    odometry.Start();

    set_term();

    SerialPort sensor_port(SENSOR_SERIAL_PORT);
    SerialPort khr_port(KHR_SERIAL_PORT);

    Motion motion(khr_port);
    std::map<int, int> dest;
    motion.Init(dest);
    motion.Move(dest);
    sleep(1);

    //motion.Clear(dest);
    //motion.Grub(dest);
    //motion.Move(dest);
    //sleep(1);

    char c, old_c, state;
    unsigned char read_buf[255];
    
    float theta, omega;

    while(1){
	sensor_port.Read(read_buf, 9);
	unsigned char sum = 0;
	for(int i = 0; i < 8; i++){
	    sum += read_buf[i];
	}

	if(sum != read_buf[8])
	    std::cout << "checksum error" << std::endl;
	else{
	    theta = *((float *)read_buf);
	    omega = *((float *)(read_buf + 4));
	}
	
	/*std::cout << std::fixed << std::setprecision(10) << theta << "\t"
	  << std::fixed << std::setprecision(10) << omega << "\t";*/

	old_c = c;
	get_key(c);

	if(c == old_c){
	    state = c;
	}

	motion.Clear(dest);

	switch(state)
	{
	case 'w':
	    std::cout << "forward";
	    motion.Forward(dest);
	    break;
	case 'a':
	    std::cout << "left";
	    motion.Left(dest);
	    break;
	case 's':
	    std::cout << "backward";
	    motion.Backward(dest);
	    break;
	case 'd':
	    std::cout << "right";
	    motion.Right(dest);
	    break;
	default :
	    std::cout << "none";
	    motion.None(dest);
	    /*if(odometry.isSet())
		motion.Stab(dest, theta);
	    else 
		motion.Stab(dest, theta, odometry.getVZ());
	    */
	    motion.Stab(dest, theta, omega, odometry.getVZ(), odometry.getZ() - 0.4);
	    //std::cout << odometry.getVZ() << std::endl;
	    break;
	}

	motion.Move(dest);
	//std::cout << "\t" << key;
	std::cout << std::endl;
    }
}

