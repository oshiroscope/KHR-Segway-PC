#pragma once
#include <opencv2/opencv.hpp>
#include <aruco.h>
#include <thread>

class Odometry{
public :
    Odometry();
    ~Odometry();
    void Start();
private :
    aruco::CameraParameters m_params;
    std::thread m_th;
};
