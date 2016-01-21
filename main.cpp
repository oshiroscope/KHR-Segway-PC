#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <thread>
#include <cmath>

#include <unistd.h>

#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"
#include "key_input.hpp"
#include "motion.hpp"
#include "odometry.hpp"
#include "monitor.hpp"

//#define DEBUG_PRINT

char key;

std::vector<char> key_buf;

int main(int argc, char *argv[])
{
    Odometry odometry;
    odometry.Start();

    KeyInput keyInput;
    keyInput.Start();

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

    std::map<int, Position> marker_map;
    float camera_theta = 0;
    std::thread control_th = std::thread(
	[&]{
	    while(1){
		marker_map = odometry.getMap();
#ifdef DEBUG_PRINT
		std::cout << marker_map[20].x << "\t"
			  << marker_map[20].y << "\t"
			  << marker_map[20].theta << "\t"
			  << atan2(-marker_map[20].x, marker_map[20].y) << "\t";
#endif

		float target_theta = atan2(-marker_map[20].x, marker_map[20].y);

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
	);

    Monitor monitor(&argc, argv, &marker_map, &camera_theta);

    monitor.Start();
    
    control_th.join();
}

