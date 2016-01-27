#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <thread>
#include <cmath>

#include <unistd.h>

#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"
#include "key_input.hpp"
#include "motion.hpp"
#include "odometry.hpp"
#include "monitor.hpp"

//#define DEBUG_PRINT
//#define KEY_CONTROL	    
#define WALK_MOTION

#define GAIN_MAX 200
#define K_DISTANCE 30.0f
#define K_DISTANCE_SUM 0.5f
#define K_VELOCITY 50.0f

char key;

std::vector<char> key_buf;

int main(int argc, char *argv[])
{
    SerialPort khr_port(KHR_SERIAL_PORT);

#ifdef WALK_MOTION
    auto cmd = CommandGen::PlayMotion({0x80, 0x9B, 0x01});
    khr_port.Write(&cmd[0], cmd.size());
    sleep(10);
#endif

    float camera_theta = 0;

    Odometry odometry(&camera_theta);
    odometry.Start();

    KeyInput keyInput;
    keyInput.Start();

    Motion motion(khr_port);
    std::map<int, int> dest;
    motion.Init(dest);
    motion.Move(dest, 40);
    sleep(1);

    motion.Clear(dest);
    motion.Push0(dest);
    motion.Move(dest, 40);
    sleep(1);

    motion.Clear(dest);
    motion.Push1(dest);
    motion.Move(dest, 40);
    sleep(1);

    motion.Clear(dest);
    motion.Push2(dest);
    motion.Move(dest, 40);
    sleep(1);

    motion.Clear(dest);
    motion.Grub(dest);
    motion.Move(dest, 40);
    sleep(1);

    std::map<int, Position> marker_map;
    Position goal;
    std::thread control_th = std::thread(
	[&]{
#ifdef KEY_CONTROL	    
	    while(1){
	    marker_map = odometry.getMap();
#ifdef DEBUG_PRINT
	    std::cout << marker_map[20].x << "\t"
		      << marker_map[20].y << "\t"
		      << marker_map[20].theta << "\t"
		      << atan2(-marker_map[20].x, marker_map[20].y) << "\t";
#endif

	    float target_theta = atan2(-marker_map[20].x, marker_map[20].y) - camera_theta;
	    motion.Clear(dest);

	    switch(keyInput.GetKey())
	    {
	case 'w':
#ifdef DEBUG_PRINT
	    std::cout << "forward";
#endif
	    motion.Forward(dest);
	    break;
	case 'a':
#ifdef DEBUG_PRINT
	    std::cout << "left";
#endif
	    motion.SetHeadOffset(1500);
	    motion.Left(dest, camera_theta);
	    break;
	case 's':
#ifdef DEBUG_PRINT
	    std::cout << "backward";
#endif
	    motion.Backward(dest);
	    break;

	case 'd':
#ifdef DEBUG_PRINT
	    std::cout << "right";
#endif
	    motion.SetHeadOffset(-1500);
	    motion.Right(dest, camera_theta);
	    break;
	default :
#ifdef DEBUG_PRINT
	    std::cout << "none";
#endif
	    motion.None(dest, camera_theta);
	    //motion.Stab(dest, theta, omega);
	    break;
	}
		
	    if((target_theta > 0.1 || target_theta < -0.1) && odometry.isSet())
		motion.Head(dest, target_theta, camera_theta);
	    motion.Move(dest, 20);

#ifdef DEBUG_PRINT
	    std::cout << std::endl;
#endif
	    usleep(30000);	
	}
#else
	    int sin_cnt = 0;
	    int blind_cnt = 0;
	    bool searched = false;

	    float distance_sum = 0.0f;
	    float old_distance = 0.0f;
	    while(1){
		motion.Clear(dest);
		
		if(!odometry.isSet() && !searched){
		    sin_cnt++;
		    motion.None(dest, 0.0f);
		    motion.Search(dest, sin(sin_cnt / 200.0f) * 2000, camera_theta);
		    motion.Move(dest, 10);
		}else if(!odometry.isSet()){
		    blind_cnt++;
		    if(blind_cnt > 20){
			motion.None(dest, camera_theta);
			motion.Move(dest, 40);
			old_distance = 0.0f;
		    }
		    if(blind_cnt > 300){
			blind_cnt = 0;
			distance_sum = 0.0f;
			searched = false;
		    }
		}else{
		    sin_cnt = 0;
		    searched = true;
		    marker_map = odometry.getMap();

		    float target_theta = atan2(-marker_map[20].x, marker_map[20].y) - camera_theta;
		    if((target_theta > 0.1 || target_theta < -0.1) && odometry.isSet())
			motion.Head(dest, target_theta, camera_theta);

		    //目標地点の算出
		    float l = 0.5f;
		    goal = {
			marker_map[20].x + l * sin(-marker_map[20].theta),
			marker_map[20].y - l * cos(-marker_map[20].theta),
			0.0f
		    };
		
		    float goal_theta = atan2(-goal.x, goal.y);// - camera_theta;
		    float distance = sqrt(goal.x * goal.x + goal.y * goal.y) * (goal.y >= 0 ? 1:-1);
		    distance_sum += distance;

		    float velocity = 0.0f;
		    if(old_distance != 0.0f){
			velocity = distance - old_distance;
			
		    }
		    old_distance = distance;
		    
		    int gain = 0;
		    
		    if(abs(distance) < 0.1){
			motion.SetHeadOffset(0);
			//motion.None(dest, camera_theta);
			distance_sum = 0.0f;
		    }else{ 
			if(goal_theta > 0.3){
			    motion.SetHeadOffset(1500);
			    motion.Left(dest, camera_theta);
			}else if(goal_theta < -0.3){
			    motion.SetHeadOffset(-1500);
			    motion.Right(dest, camera_theta);
			}else{
			    motion.SetHeadOffset(0);
			    motion.None(dest, camera_theta);
			}
			gain = distance * K_DISTANCE + distance_sum * K_DISTANCE_SUM - velocity * K_VELOCITY;
			if(gain > GAIN_MAX) gain = GAIN_MAX;
			if(gain < -GAIN_MAX) gain = -GAIN_MAX;
			motion.StraightCtrl(dest, gain); 
		    }
		    std::cout <<distance << "\t" << distance_sum << "\t" << velocity << "\t" <<  gain << "\n";		
		    // if(marker_map[20].theta > 0.7){
		    //     motion.SetHeadOffset(-1500);
		    //     motion.Right(dest, camera_theta);
		    // }else if(marker_map[20].theta < -0.7){
		    //     motion.SetHeadOffset(1500);
		    //     motion.Left(dest, camera_theta);
		    // }else{
		    //     motion.None(dest, camera_theta);
		    // }

		    motion.Move(dest, 30);
		}
		usleep(20000);	
	    }
#endif
	}
	);

    Monitor monitor(&argc, argv, &marker_map, &camera_theta, &goal);

    monitor.Start();
    while(1){}
    control_th.join();
}

