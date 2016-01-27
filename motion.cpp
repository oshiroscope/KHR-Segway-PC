#include "motion.hpp"

#include <map>
#include <iostream>
#include <cmath>

#include "command_gen.hpp"

/*
  NECK = 0, //首(壊れている?)
  WAIST = 1, //腰(回転方向不明)
  LEFT_SHOULDER_PITCH = 2, //左肩ピッチ方向回転
  RIGHT_SHOULDER_PITCH = 3, //右肩ピッチ方向回転
  LEFT_SHOULDER_ROLL = 4, //左肩ロール回転(外側に開く)
  RIGHT_SHOULDER_ROLL = 5, //右肩ロール回転(内側に閉じる(鏡面対称ではない))
  LEFT_SHOULDER_YAW = 6, //左肩ヨー方向回転
  RIGHT_SHOULDER_YAW = 7, //右肩ヨー方向回転
  LEFT_ELBOW = 8, //左肘
  RIGHT_ELBOW = 9, //右肘
  LEFT_HIP_YAW = 10, //左股間節ヨー方向回転
  RIGHT_HIP_YAW = 11, //右股間節ヨー方向回転
  LEFT_HIP_ROLL = 12, //左股関節ロール方向回転
  RIGHT_HIP_ROLL = 13, //右股関節ロール方向回転
  LEFT_HIP_PITCH = 14, //左股関節ピッチ方向回転
  RIGHT_HIP_PITCH = 15, //右股関節ピッチ方向回転
  LEFT_KNEE = 16, //左膝
  RIGHT_KNEE = 17, //右膝
  LEFT_ANKLE_PITCH = 18, //左足首ピッチ方向回転
  RIGHT_ANKLE_PITCH = 19, //右足首ピッチ方向回転
  LEFT_ANKLE_ROLL = 20, //左足首ロール方向回転
  RIGHT_ANKLE_ROLL = 21, //右足首ロール方向回転
*/

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

const int INIT_0 = 7500;
const int INIT_2 = 8500;
const int INIT_3 = 6500;
const int INIT_6 = 7500;
const int INIT_7 = 7500;
const int INIT_8 = 7000;
const int INIT_9 = 8000;
const int INIT_14 = 7500;
const int INIT_15 = 7500;
const int INIT_16 = 7500;
const int INIT_17 = 7500;
const int INIT_18 = 7560;
const int INIT_19 = 7440;

const int GRUB_4 = 7200;
const int GRUB_5 = 7800;
const int GRUB_6 = 5000;
const int GRUB_7 = 10000;
const int GRUB_8 = 7000;
const int GRUB_9 = 8000;

void Motion::Init(std::map<int, int> &dest){
    dest[0] = INIT_0;//7500;
    dest[2] = INIT_2;// 8500; //8000; //7300;
    dest[3] = INIT_3;// 6500; //7000; //7700;
    dest[6] = INIT_6;// 7500; //8000;
    dest[7] = INIT_7;// 7500; //7000;
    dest[8] = INIT_8;// 6000; //6000; //4700;
    dest[9] = INIT_9;// 9000; //9000; //10300;
    
    dest[14] = INIT_14;
    dest[15] = INIT_15;
    dest[16] = INIT_16;
    dest[17] = INIT_17;
    
    dest[18] = INIT_18;
    dest[19] = INIT_19;
}

void Motion::Push0(std::map<int, int> &dest){
    dest[3] = INIT_3;
    dest[9] = INIT_9 + 2000;
}

void Motion::Push1(std::map<int, int> &dest){
    dest[2] = INIT_2;// 8000; //7300;
    dest[3] = INIT_3;// 7000; //7700;
    dest[5] = GRUB_5; //6000; //8000;
    dest[7] = GRUB_7; //9000; //7000;
    dest[9] = INIT_9 + 2000; //10000; //9000; //10300;
    dest[18] = INIT_18;
    dest[19] = INIT_19;
}

void Motion::Push2(std::map<int, int> &dest){
    dest[9] = GRUB_9;// 9000; //9000; //10300;    
}

void Motion::Push3(std::map<int, int> &dest){
    dest[9] = INIT_9 - 1000;// 9000; //9000; //10300;    
}

void Motion::Grub(std::map<int, int> &dest){
    dest[2] = INIT_2;// 8000; //7300;
    dest[3] = INIT_3;// 7000; //7700;
    dest[4] = GRUB_4; //6000; //8000;
    dest[5] = GRUB_5; //6000; //8000;
    dest[6] = GRUB_6; //6000; //8000;
    dest[7] = GRUB_7; //9000; //7000;
    dest[8] = GRUB_8; //5000; //6000; //4700;
    dest[9] = GRUB_9; //10000; //9000; //10300;
    dest[18] = INIT_18;
    dest[19] = INIT_19;
}

void Motion::Left(std::map<int, int> &dest, float camera_theta){
    if(!m_head_changed){
	//dest[0] = 9000;
	dest[0] = Rad2Servo(-camera_theta) + m_head_offset;
	m_head_changed = true;
    }

    //SetHeadOffset(1500);

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

void Motion::Right(std::map <int, int> &dest, float camera_theta){
    if(!m_head_changed){
	//dest[0] = 6000;
	dest[0] = Rad2Servo(-camera_theta) + m_head_offset;
	m_head_changed = true;
    }

    
    //SetHeadOffset(-1500);

    dest[1] = 7500 + 1500;

    /*dest[3] = 7700 - 200;
      dest[7] = 7500 - 600;
      dest[9] = 10300 - 400;

      dest[2] = 7300 - 200;
      dest[6] = 7500 - 600;
      dest[8] = 4700 - 200;*/
}

void Motion::StraightCtrl(std::map<int, int> &dest, int gain){
    dest[18] = INIT_18 - gain;
    dest[19] = INIT_19 + gain;
}

void Motion::Forward(std::map<int, int> &dest){
    dest[18] = INIT_18 - F_OFFSET;
    dest[19] = INIT_19 + F_OFFSET;
    // dest[2] = 7500 + 800;
    // dest[8] = 4700 + 1200;

    // dest[3] = 7500 - 800;
    // dest[9] = 10300 - 1200;    
    
    // dest[18] = 7500 + 80;
    // dest[19] = 7500 - 80;
}

void Motion::Backward(std::map<int, int> &dest){
    dest[18] = INIT_18 + B_OFFSET;
    dest[19] = INIT_19 - B_OFFSET;
    // dest[2] = 7500 - 1400;
    // dest[8] = 4700 - 1200;

    // dest[3] = 7500 + 1400;
    // dest[9] = 10300 + 1200;    

    // dest[18] = 7500 + 10;
    // dest[19] = 7500 - 10;
}

void Motion::None(std::map<int, int> &dest, float camera_theta){
    SetHeadOffset(0);
    if(m_head_changed){
	dest[0] = Rad2Servo(-camera_theta) + m_head_offset;

	m_head_changed = false;
    }
    dest[1] = 7500;

    dest[2] = INIT_2;// 8000; //7300;
    dest[3] = INIT_3;// 7000; //7700;
    dest[6] = GRUB_6; //6000; //8000;
    dest[7] = GRUB_7; //9000; //7000;
    dest[8] = GRUB_8; //5000; //6000; //4700;
    dest[9] = GRUB_9; //10000; //9000; //10300;

    dest[12] = 7500;
    dest[13] = 7500;
    dest[20] = 7500;
    dest[21] = 7500;

    dest[18] = INIT_18;
    dest[19] = INIT_19;
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
int Motion::Rad2Servo(float angle){
    return 7500 +  angle * 5333 / M_PI; // 135 deg = 4000 unit / 3.141592rad = 5333
}
float Motion::Servo2Rad(int angle){
    return (angle - 7500) * M_PI / (float)5333;
}

void Motion::SetHeadOffset(int offset){
    m_head_offset = offset;
}

void Motion::Head(std::map<int, int> &dest, float target_theta, float &camera_theta){
    dest[0] = ((Rad2Servo(-camera_theta) - target_theta * 30)) + m_head_offset;
    
    /*std::cout << m_head_offset << "\t"
	      << dest[0] << "\t"
	      << Rad2Servo(-camera_theta) << "\t"
	      << camera_theta << "\t"
	      << target_theta << "\n";*/
    
    camera_theta = -Servo2Rad(dest[0] - m_head_offset);

    if(dest[0] > 11000)
	dest[0] = 11000;
    if(dest[0] < 4000)
	dest[0] = 4000;
}

void Motion::Search(std::map<int, int> &dest, int head_input, float &camera_theta){
    dest[0] = head_input + 7500;
    camera_theta = -Servo2Rad(dest[0]);
}

void Motion::Clear(std::map <int, int> &dest){
    dest.clear();
}

