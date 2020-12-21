#include <std_msgs/Int64MultiArray.h>
#include <ros/ros.h>
#include <termios.h>
#include <iostream>

#define STD_SPEED   0
#define STD_TURN    90
#define DIFF_TURN   30  

static struct termios oldt;  

struct MobileParam {
    int speed,
        turn;
        
    MobileParam(int speed_ = 0, int turn_ = 0):
        speed(speed_),
        turn(turn_)
    {
    }
};

void help() {
    std::cout << "Control Your mobile!\n"
            << "------------------------------\n"
            << "space key, k: force stop\n"
            << "w/s: shift the middle pos of throttle by +/- 5 pwm\n"
            << "a/d: shift the middle pos of steering by +/- 2 pwm\n"
            << "Ctrl+C to quit\n";
}

void constrain(int& x, int min, int max) {
    x = (x < min ? min : (x > max ? max : x));
}

void print(MobileParam param) {
	ROS_INFO("currently:\n\tspeed %d\n\tturn: %d\n", param.speed, param.turn);
}

bool is_readable() {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    select(1, &fds, NULL, NULL, &tv);
    return (FD_ISSET(0, &fds));
}

int getch() {
	int r;
	unsigned char c;
	if ((r = read(0, &c, sizeof(c))) < 0)
		return r;
	else
		return c;
}


// ======================
// =====    MAIN    =====
// ======================
int main(int argc, char** argv) {
    // ROS
    ros::init(argc, argv, "mobile_teleop");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::Int64MultiArray>("mobile/cmd_vel", 1);
    ros::Rate rate(50.);
    // Другие параметры
	bool end = false;
    MobileParam param_control(STD_SPEED, STD_TURN);
    
    // Изменяем атрибуты терминала
    struct termios newt;
    tcgetattr(0, &oldt);
    newt = oldt;
    atexit([](){
         tcsetattr(0, TCSANOW, &oldt);
    });
    newt.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP
                            |INLCR|IGNCR|ICRNL|IXON);
    newt.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    tcsetattr(0, TCSANOW, &newt);

    help();
    
    while(ros::ok() && !end) {
        if(is_readable()) {
        	char key = getch();
        	switch(key) {
            	case ' ':
            	case 'k':
                	param_control.speed = STD_SPEED;
                	param_control.turn = STD_TURN;
            	break;
                
            	case 'w':
                	param_control.speed += 5;
            	break;
            
            	case 's':
                	param_control.speed -= 5;
            	break;
            
            	case 'a':
                	param_control.turn += 2;
           		break;
            
            	case 'd':
                	param_control.turn -= 2;
            	break;
            	case '\x003':
                	end = true;
                	return 0;
            	break;
        	}
        	constrain(param_control.speed, 0, 255);
        	constrain(param_control.turn, STD_TURN - DIFF_TURN, STD_TURN + DIFF_TURN);
        }
    	std_msgs::Int64MultiArray cmd;
		cmd.data.push_back(param_control.speed);
		cmd.data.push_back(param_control.turn);
		publisher.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
