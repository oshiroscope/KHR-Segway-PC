#pragma once
#include <GL/glut.h>
#include <map>
#include "odometry.hpp"


class Monitor{
public:
    Monitor(int *argc, char *argv[], std::map<int, Position> *marker_map, float *camera_theta, Position *goal);
    ~Monitor();

    void Start();
private:    
    static void display();
};
