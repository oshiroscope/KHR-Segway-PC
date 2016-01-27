#pragma once

#include <vector>
#include <map>

namespace CommandGen{
    std::vector<unsigned char> SeriesServoMove(std::map<int, int> dest, int frame);
    std::vector<unsigned char> SetFree(int id);
    std::vector<unsigned char> PlayMotion(std::vector<unsigned char> addr);
}

