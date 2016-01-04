#include "command_gen.hpp"

std::vector<unsigned char> CommandGen::SeriesServoMove(std::map<int, int> dest, int frame)
{
    int cnt = dest.size();

    unsigned char size = (unsigned char)(8 + 2 * cnt + 1);
    std::vector<unsigned char> cmd((int)size);
    cmd[0] = size;
    cmd[1] = 0x10;
    cmd[7] = (unsigned char)frame;

    int select_index = 2;
    int dest_index = 8;
    int servo_cnt = 0;

    for(auto i : dest)
    {
	int quot = (int)(i.first / 8);
	int offset = i.first % 8;
	cmd[select_index + quot] += (unsigned char)(0x01 << offset);

	cmd[dest_index + servo_cnt * 2] = i.second & 0xFF;
	cmd[dest_index + servo_cnt * 2 + 1] = i.second >> 8;
	servo_cnt++;
    }
    
    unsigned char sum = 0;
    for(int i = 0; i < (int)size - 1; i++)
    {
	sum += cmd[i];
    }
    cmd[size - 1] = sum;

    return cmd;
}

std::vector<unsigned char> CommandGen::SetFree(int id)
{
    int size = 9;
    std::vector<unsigned char> cmd(size);

    cmd[0] = (unsigned char)size;
    cmd[1] = 0x00;
    cmd[2] = 0b00000010; // literal -> ram
    unsigned int addr = 0x044 + 20 * id + 6;
    cmd[3] = addr & 0xFF;
    cmd[4] = addr >> 8;
    cmd[5] = 0x00;
    cmd[6] = 32767 & 0xFF;
    cmd[7] = 32767 >> 8;
    cmd[8] = 0x00;
    
    for(int i = 0; i < size; i++)
    {
	cmd[8] += cmd[i];
    }
    return cmd;
}
