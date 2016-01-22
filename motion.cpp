#include "motion.hpp"

#include <map>
#include <iostream>
#include <cmath>

#include "command_gen.hpp"



Motion::Motion(SerialPort khr_port)
    : m_khr_port(khr_port){}

void Motion::Move(std::map<int, int> &dest)
{
    auto cmd = CommandGen::SeriesServoMove(dest, 5);
    m_khr_port.Write(&cmd[0], cmd.size());
}

void Motion::Move(std::map<int, int> &dest, int frame)
{
    auto cmd = CommandGen::SeriesServoMove(dest, frame);
    m_khr_port.Write(&cmd[0], cmd.size());
}

void Motion::Init(std::map<int, int> &dest){
    dest[0] = 7500;
    dest[2] = 8000; //7300;
    dest[3] = 7000; //7700;
    dest[6] = 7500; //8000;
    dest[7] = 7500; //7000;
    dest[8] = 6000; //4700;
    dest[9] = 9000; //10300;
}

void Motion::Grub(std::map<int, int> &dest){
    dest[6] = 7000; //7500;
    dest[7] = 8000; //7500;
}

void Motion::Left(std::map<int, int> &dest){
    m_head_changed = true;
    //SetHeadOffset(1500);
    //dest[0] = 9000;
    dest[1] = 7500 - 1500;

    //dest[7] = 9000;
    //dest[9] = 10000;

    /*dest[3] = 7700 + 200;
    dest[7] = 7500 + 600;
    dest[9] = 10300 + 200;

    dest[2] = 7300 + 200;
    dest[6] = 7500 + 600;
    dest[8] = 4700 + 400;*/
}

void Motion::Right(std::map <int, int> &dest){
    m_head_changed = true;
    //SetHeadOffset(-1500);
    //dest[0] = 6000;
    dest[1] = 7500 + 1500;

    /*dest[3] = 7700 - 200;
    dest[7] = 7500 - 600;
    dest[9] = 10300 - 400;

    dest[2] = 7300 - 200;
    dest[6] = 7500 - 600;
    dest[8] = 4700 - 200;*/
}

void Motion::Forward(std::map<int, int> &dest){
    dest[18] = 7500 - F_OFFSET;
    dest[19] = 7500 + F_OFFSET;
    // dest[2] = 7500 + 800;
    // dest[8] = 4700 + 1200;

    // dest[3] = 7500 - 800;
    // dest[9] = 10300 - 1200;    
    
    // dest[18] = 7500 + 80;
    // dest[19] = 7500 - 80;
}

void Motion::Backward(std::map<int, int> &dest){
    dest[18] = 7500 + B_OFFSET;
    dest[19] = 7500 - B_OFFSET;
    // dest[2] = 7500 - 1400;
    // dest[8] = 4700 - 1200;

    // dest[3] = 7500 + 1400;
    // dest[9] = 10300 + 1200;    

    // dest[18] = 7500 + 10;
    // dest[19] = 7500 - 10;
}

void Motion::None(std::map<int, int> &dest){
    SetHeadOffset(0);
    if(m_head_changed){
	dest[0] = 7500;
	m_head_changed = false;
    }
    dest[1] = 7500;


    dest[2] = 8000; //7300;
    dest[3] = 7000; //7700;
    dest[6] = 6000; //8000;
    dest[7] = 9000; //7000;
    dest[8] = 6000; //4700;
    dest[9] = 9000; //10300;

    dest[12] = 7500;
    dest[13] = 7500;
    dest[20] = 7500;
    dest[21] = 7500;

    dest[18] = 7500;
    dest[19] = 7500;
}

void Motion::Stab(std::map<int, int> &dest, float theta){
    dest[18] = (float)7500 + (theta - THETA_INIT) * (float)100;
    dest[19] = (float)7500 - (theta - THETA_INIT) * (float)100;
}

void Motion::Stab(std::map<int, int> &dest, float theta, float omega){
    dest[18] = (float)7500 + (theta - THETA_INIT) * (float)300 + omega * 50;
    dest[19] = (float)7500 - (theta - THETA_INIT) * (float)300 - omega * 50;
}

void Motion::Stab(std::map<int, int> &dest, float theta, float v, float x){
    //dest[18] = (float)7500 + (theta - THETA_INIT) * (float)1500;
    //dest[19] = (float)7500 - (theta - THETA_INIT) * (float)1500;
    dest[18] = (float)7500 + (theta - THETA_INIT) * (float)1000 - x * 150;
    dest[19] = (float)7500 - (theta - THETA_INIT) * (float)1000 + x * 150;
    //std::cout << v * 150 << "\t" << x << std::endl;
}

void Motion::Stab(std::map<int, int> &dest, float theta, float omega, float v, float x){
    //dest[18] = (float)7500 + (theta - THETA_INIT) * (float)1500;
    //dest[19] = (float)7500 - (theta - THETA_INIT) * (float)1500;
    dest[18] = (float)7500 + (theta - THETA_INIT) * (float)100 + omega * 50 - x * 20 - v * 30;
    dest[19] = (float)7500 - (theta - THETA_INIT) * (float)100 - omega * 50 + x * 20 + v * 30;
    /*std::cout << (theta - THETA_INIT)*200 << "\t" 
	      << omega * 100 << "\t" 
	      << x * 300 << "\t" 
	      << v * 100 << std::endl;*/
}

//
int Rad2Servo(float angle){
    return 7500 +  angle * 5333 / M_PI; // 135 deg = 4000 unit / 3.141592rad = 5333
}
float Servo2Rad(int angle){
    return (angle - 7500) * M_PI / (float)5333;
}

void Motion::SetHeadOffset(int offset){
    m_head_offset = offset;
}

void Motion::Head(std::map<int, int> &dest, float target_theta, float &camera_theta){
    if(m_head_changed)
	return ;
    dest[0] =((Rad2Servo(-camera_theta) - target_theta * 30)) + m_head_offset;
    std::cout << Rad2Servo(-camera_theta) << "\t"
	      << camera_theta << "\t"
	      << target_theta * 50 << "\n";
    
    camera_theta = -Servo2Rad(dest[0] - m_head_offset);

    if(dest[0] > 11000)
	dest[0] = 11000;
    if(dest[0] < 4000)
	dest[0] = 4000;
}

void Motion::Clear(std::map <int, int> &dest){
    dest.clear();
}

