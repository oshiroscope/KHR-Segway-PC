#pragma once

#include <vector>
#include <map>

#include "config.hpp"
#include "serial_port.hpp"

class Motion{
public:
    Motion(SerialPort khr_port);
    
    void Move(std::map<int, int> &dest);
    void Move(std::map<int, int> &dest, int frame);
    void Init(std::map<int, int> &dest);
    void Grub(std::map<int, int> &dest);
    void Left(std::map<int, int> &dest, float camera_theta);
    void Right(std::map<int, int> &dest, float camera_theta);
    void Forward(std::map<int, int> &dest);
    void Backward(std::map<int, int> &dest);
    void None(std::map<int, int> &dest, float camera_theta);
    void Stab(std::map<int, int> &dest, float theta);
    void Stab(std::map<int, int> &dest, float theta, float omega);
    void Stab(std::map<int, int> &dest, float theta, float v, float x);
    void Stab(std::map<int, int> &dest, float theta, float omega, float v, float x);
    void SetHeadOffset(int offset);
    void Head(std::map<int, int> &dest, float target_theta, float &camera_theta);
    void Clear(std::map<int, int> &dest);

private:
    SerialPort m_khr_port;
    int m_head_offset = 0;
    bool m_head_changed = false;
    int Rad2Servo(float angle);
    float Servo2Rad(int angle);

};



