#pragma once
#include <opencv2/opencv.hpp>
#include <aruco.h>
#include <thread>

class Odometry{
public :
    Odometry();
    ~Odometry();
    void Start();
    float getX(){return m_x;}
    float getY(){return m_y;}
    float getZ(){return m_z;}
    float getVX(){return m_vx;}
    float getVY(){return m_vy;}
    float getVZ(){return m_vz;}
    bool isSet(){return m_is_set;}
private :
    aruco::CameraParameters m_params;
    std::thread m_th;
    float m_x;
    float m_y;    
    float m_z;
    float m_vx;
    float m_vy;
    float m_vz;
    bool m_is_set;
    
};
