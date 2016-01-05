#include "motion.hpp"

#include <map>

#include "command_gen.hpp"

Motion::Motion(SerialPort khr_port)
    : m_khr_port(khr_port){}

void Motion::Move(std::map<int, int> &dest)
{
    auto cmd = CommandGen::SeriesServoMove(dest, 10);
    m_khr_port.Write(&cmd[0], cmd.size());
}

void Motion::Init(std::map<int, int> &dest){
    dest[2] = 7300;
    dest[3] = 7700;
    dest[6] = 8000;
    dest[7] = 7000;
    dest[8] = 4700;
    dest[9] = 10300;
}

void Motion::Grub(std::map<int, int> &dest){
    dest[6] = 7500;
    dest[7] = 7500;
}

void Motion::Left(std::map<int, int> &dest){
    dest[1] = 7500 - 400;

    dest[3] = 7700 + 200;
    dest[7] = 7500 + 600;
    dest[9] = 10300 + 200;

    dest[2] = 7300 + 200;
    dest[6] = 7500 + 600;
    dest[8] = 4700 + 400;
}

void Motion::Right(std::map <int, int> &dest){
    dest[1] = 7500 + 400;

    dest[3] = 7700 - 200;
    dest[7] = 7500 - 600;
    dest[9] = 10300 - 400;

    dest[2] = 7300 - 200;
    dest[6] = 7500 - 600;
    dest[8] = 4700 - 200;
}

void Motion::Forward(std::map<int, int> &dest){
    dest[18] = 7500 - F_OFFSET;
    dest[19] = 7500 + F_OFFSET;
}

void Motion::Backward(std::map<int, int> &dest){
    dest[18] = 7500 + B_OFFSET;
    dest[19] = 7500 - B_OFFSET;
}

void Motion::None(std::map<int, int> &dest){
    dest[1] = 7500;
    dest[3] = 7700;
    dest[7] = 7500;
    dest[9] = 10300;
    
    dest[2] = 7300;
    dest[6] = 7500;
    dest[8] = 4700;
    
    dest[12] = 7500;
    dest[13] = 7500;
    dest[20] = 7500;
    dest[21] = 7500;

    dest[18] = 7500;
    dest[19] = 7500;
}

void Motion::Stab(std::map<int, int> &dest, float theta){
    dest[18] = (float)7500 + (theta - THETA_INIT) * (float)1500;
    dest[19] = (float)7500 - (theta - THETA_INIT) * (float)1500;
}

void Motion::Clear(std::map <int, int> &dest){
    dest.clear();
}

