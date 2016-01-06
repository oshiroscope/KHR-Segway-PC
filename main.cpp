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

char key;

int main()
{
    aruco::CameraParameters params;
    params.readFromXMLFile("./intrinsics.yml");
    //params.resize(inputImage.size());
    
    aruco::MarkerDetector detector;
    std::vector<aruco::Marker> markers;
    const float markerSize = 0.04f;

    cv::VideoCapture cap(1);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    if(!cap.isOpened())
    {
	return -1;
    }
    cv::namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while(1){
	cv::Mat frame;
	cap >> frame;

	detector.detect(frame, markers, params, markerSize);

	auto outputImage = frame.clone();
	for(auto&& marker : markers){
	    std::cout << marker << std::endl;
	    marker.draw(outputImage, cv::Scalar(0, 0, 255), 2);
	    aruco::CvDrawingUtils::draw3dCube(outputImage, marker, params);
	}
	cv::imshow("Capture", outputImage);
	if(cv::waitKey(30) >= 0)
        {
            break;
        }
    }
    return 0;
    
    set_term();

    SerialPort sensor_port(SENSOR_SERIAL_PORT);
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

    while(1){
	sensor_port.Read(read_buf, 7);
	float theta = *((float *)read_buf);
	std::cout << std::fixed << std::setprecision(10) << theta << "\t";

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
	    motion.Stab(dest, theta);
	    break;
	}

	motion.Move(dest);

	std::cout << "\t" << key;
	std::cout << std::endl;
    }
}

