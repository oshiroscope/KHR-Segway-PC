#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <thread>
#include <cmath>

#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <aruco.h>

#include <GL/glut.h>

#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"
#include "key_input.hpp"
#include "motion.hpp"
#include "odometry.hpp"

//#define DEBUG_PRINT

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

    
    float camera_theta = 0;

    while(1){
	auto map = odometry.getMap();
	#ifdef DEBUG_PRINT
	std::cout << map[20].x << "\t"
		  << map[20].y << "\t"
		  << map[20].theta << "\t"
		  << atan2(-map[20].x, map[20].y) << "\t";
	#endif

	float target_theta = atan2(-map[20].x, map[20].y);

	motion.Clear(dest);

	switch(keyInput.GetKey())
	{
	case 'w':
#ifdef DEBUG_PRINT
	    std::cout << "forward";
#endif
	    motion.Forward(dest);
	    break;
	case 'a':
#ifdef DEBUG_PRINT
	    std::cout << "left";
#endif
	    motion.Left(dest);
	    break;
	case 's':
#ifdef DEBUG_PRINT
	    std::cout << "backward";
#endif
	    motion.Backward(dest);
	    break;
	case 'd':
#ifdef DEBUG_PRINT
	    std::cout << "right";
#endif
	    motion.Right(dest);
	    break;
	default :
#ifdef DEBUG_PRINT
	    std::cout << "none";
#endif
	    motion.None(dest);
	    //motion.Stab(dest, theta, omega);
	    break;
	}

	if(target_theta > 0.1 || target_theta < -0.1)
	    motion.Head(dest, target_theta, camera_theta);
	motion.Move(dest, 40);
	
#ifdef DEBUG_PRINT
	std::cout << std::endl;
#endif
	usleep(30000);	
    }
}

