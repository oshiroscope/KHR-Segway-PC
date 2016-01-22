#include "monitor.hpp"

#include <iostream>
#include <cmath>

std::map<int, Position> *g_marker_map;
float *g_camera_theta;

void render_string(float x, float y, std::string str){
    float z = -1.0f;
    glRasterPos3f(x, y, z);
    char* p = (char*) str.c_str();
    while (*p != '\0') glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p++);
}

void render_vector(float x, float y, float theta, float length, float width)
{
    glLineWidth(width);
    glBegin(GL_LINE_LOOP);
    
    glVertex2d(x, y);
    glVertex2d(x + length * cos(theta + M_PI/2),
	       y + length * sin(theta + M_PI/2));
    glEnd();

    glBegin(GL_POLYGON);
    glColor4f(0.7,0.2,0.2,0.0);
    double cx, cy, cr = 0.015;
    int n = 20;
    for(int i = 0; i < n; i++){
	cx = x + cr * cos(2.0 * 3.14 * ((double)i/n) );
	cy = y + cr * sin(2.0 * 3.14 * ((double)i/n) );
	glVertex3f(cx, cy, 0.0);
    }
    glEnd();
}

const int scale = 2.5;

void Monitor::display(){
    glClear(GL_COLOR_BUFFER_BIT); 

    for(auto&& marker : *g_marker_map){
	float pos_x = marker.second.x / scale;
	float pos_y = marker.second.y / scale - 1;	
	glColor4f(0.8,0.2,0.2,0.0);
	render_vector(pos_x, pos_y, -marker.second.theta, 0.2, 1.5);
	glColor4f(0.4,0.8,1.0,0.0);
	render_string(pos_x + 0.05, pos_y, "id =" + to_string(marker.first)); 
	glColor4f(0.8,0.8,0.2,0.0);
	render_vector(0.0, -1.0, *g_camera_theta, 0.2, 3);
	glColor4f(1.0,1.0,1.0,0.0);	
	render_vector(0.0, -1.0, *g_camera_theta + 0.25, 2, 1.5);
	glColor4f(1.0,1.0,1.0,0.0);	
	render_vector(0.0, -1.0, *g_camera_theta - 0.25, 2, 1.5);
    }


    glFlush();
}

void idle(){
    glutPostRedisplay();
}

void init(){
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glutIdleFunc(idle);
}

Monitor::Monitor(int *argc, char *argv[], std::map<int, Position> *marker_map, float *camera_theta){
    g_marker_map = marker_map;
    g_camera_theta = camera_theta;
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA);
    glutInitWindowSize(1000, 1000);
    glutInitWindowPosition(-1500, 500);
    glutCreateWindow(argv[0]);
}

Monitor::~Monitor()
{
}

void Monitor::Start()
{
    std::cout << "start gl" << std::endl;
    glutDisplayFunc(this->display);
    init();
    glutMainLoop();
}
