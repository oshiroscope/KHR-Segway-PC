#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "serial_port.hpp"
#include "command_gen.hpp"

char key;

static void post_proc(int);
static struct termios SavedTermAttr;

int main()
{
    signal(SIGINT, &post_proc);
    struct termios term_attr;

    /* Terminal control */
    /* ファイルディスクリプタ０(標準入力)
       のターミナル属性を取得 */
    if( tcgetattr(0, &term_attr) < 0 ){
	fprintf(stderr, "Can't get terminal attributes.\n");
	post_proc(-1);
    }
    /* 取得したターミナル属性を保存 */
    SavedTermAttr = term_attr;
    /* ICANON、ECHOフラッグをセット */
    term_attr.c_lflag &= ~(ICANON|ECHO);
    /* 入力文字列の最小読出バイト数を１に */
    term_attr.c_cc[VMIN] = 1; 
    /* 入力待ち時間を０に */
    term_attr.c_cc[VTIME] = 0; 

    /* 変更したターミナル属性をセット */
    if( tcsetattr(0, TCSANOW, &term_attr) < 0 ){ 
	fprintf(stderr, "Can't change terminal attributes.\n");
	post_proc(-1);
    }
    /* NONBLOCKフラグのセット */
    if( fcntl(0, F_SETFL, O_NONBLOCK) == -1 ){
	fprintf(stderr, "Can't fcntl().\n");
	post_proc(-1);
    }

    SerialPort sensor_port(SENSOR_SERIAL_PORT);
    SerialPort khr_port(KHR_SERIAL_PORT);    
    unsigned char read_buf[255];

    std::map <int, int> dest;
    dest[2] = 7300;
    dest[3] = 7700;
    dest[6] = 8000;
    dest[7] = 7000;
    dest[8] = 4700;
    dest[9] = 10300;
    auto cmd = CommandGen::SeriesServoMove(dest, 20);
    khr_port.Write(&cmd[0], cmd.size());
    sleep(1);
    
    dest.clear();
    dest[6] = 7500;
    dest[7] = 7500;
    auto cmd2 = CommandGen::SeriesServoMove(dest, 20);
    khr_port.Write(&cmd2[0], cmd2.size());
    sleep(2);

    dest.clear();

    char c, old_c, state;

    while(1){
	sensor_port.Read(read_buf, 7);
	float theta = *((float *)read_buf);
	std::cout << std::fixed << std::setprecision(10) << theta << "\t";

	old_c = c;
	if(read(0, &c, 1) == 1 )
	{
	    if( errno != EAGAIN )
	    {
		fprintf(stderr, "EOF\n");
		post_proc(0);
	    }
	}
	else
	{
	    c = 0x00;
	}

	std::map <int, int> dest;
	dest[18] = (float)7500 + (theta - THETA_INIT) * (float)1500;
	dest[19] = (float)7500 - (theta - THETA_INIT) * (float)1500;
	
	if(c == old_c){
	    state = c;
	}
	switch(state)
	{
	case 'w':
	    std::cout << "forward";
	    dest[18] = 7500 - F_OFFSET;
	    dest[19] = 7500 + F_OFFSET;
	    break;
	case 'a':
	    std::cout << "left";
	    dest[1] = 7500 - 400;

	    dest[3] = 7700 + 200;
	    dest[7] = 7500 + 600;
	    dest[9] = 10300 + 200;

	    dest[2] = 7300 + 200;
	    dest[6] = 7500 + 600;
	    dest[8] = 4700 + 400;
	    break;
	case 's':
	    std::cout << "backward";
	    dest[18] = 7500 + B_OFFSET;
	    dest[19] = 7500 - B_OFFSET;
	    break;
	case 'd':
	    std::cout << "right";
	    dest[1] = 7500 + 400;

	    dest[3] = 7700 - 200;
	    dest[7] = 7500 - 600;
	    dest[9] = 10300 - 400;

	    dest[2] = 7300 - 200;
	    dest[6] = 7500 - 600;
	    dest[8] = 4700 - 200;
	    break;
	default :
	    std::cout << "none";
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
	    break;
	}

	auto cmd = CommandGen::SeriesServoMove(dest, 5);
	khr_port.Write(&cmd[0], cmd.size());
	std::cout << "\t" << key;
	std::cout << std::endl;
    }

}

void post_proc(int sig)
{
/* 保存されたターミナル属性を復元 */
    if( tcsetattr(0, TCSANOW, &SavedTermAttr) < 0 ){
	fprintf(stderr, "Can't change terminal attribute.\n");
	exit(-1);
    }
    if( sig < 0 )
	exit(-1);
    else
	exit(0);
}
