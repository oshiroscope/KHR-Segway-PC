#pragma once
#include <opencv2/opencv.hpp>
#include <aruco.h>
#include <thread>

struct Vec3f{
public:
    float x;
    float y;
    float z;
};

struct Coord{
public:
    Vec3f transform;
    Vec3f rotation;
};

struct Position{
public:
    float x;
    float y;
    float theta;
};

class Odometry{
public :
    Odometry(float *camera_theta);
    ~Odometry();
    void Start();
    float getX(){return m_x;}
    float getY(){return m_y;}
    float getZ(){return m_z;}
    float getVX(){return m_vx;}
    float getVY(){return m_vy;}
    float getVZ(){return m_vz;}
    std::map<int, Position> getMap(){return m_marker_vec;}
    bool isSet(){return m_is_set;}
private :
    aruco::CameraParameters m_params;
    std::thread m_th;
    std::map<int, Position> m_marker_vec;
    float m_x;
    float m_y;    
    float m_z;
    float m_vx;
    float m_vy;
    float m_vz;
    bool m_is_set;
    float *m_camera_theta;
};
