#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <termios.h>
#include <iostream>

static struct termios oldt;  

struct MobileParam {
    float speed,
        turn;
        
    MobileParam(float speed_ = 0.0, float turn_ = 0.0):
        speed(speed_),
        turn(turn_)
    {
    }
};

void help_param(char* name) {
	std::cout << "Use: \n" 
			  << "	" << name << " <ros remapping arguments> <wheelbase> <max backward speed> <max forward speed> <steering range>\n"
			  << "Options: \n"
			  << "	<wheelbase>				  --		wheelbase in m (default: 1.0 m)\n"
			  << "	<max backward speed> 	  --		max backward speed in m/s (default: 0.5 m/s)\n"
			  << "										if set to a negative value, then it will become 0.0 m/s\n"
			  << "	<max forward speed>		  --		max forward speed in m/s (default: 1.0 m/s)\n"
			  << "										if set to a negative value, then it will become 0.0 m/s\n"
			  << "	<steering range>					steering range in degrees (default: 30 degrees)\n"
			  << "							  --		if set to a negative value, then it will become 0.0 m/s\n"
			  << "										minimum border calculate: 0 - <steering range>\n"
			  << "										maximum border calculate: 0 + <steering range>\n"
}

void help() {
    std::cout << "Control Your mobile!\n"
            << "------------------------------\n"
            << "space key, k: force stop\n"
            << "w/s: shift the middle pos of throttle by +/- 0.05 m/s\n"
            << "a/d: shift the middle pos of steering by +/- 1 degree\n"
            << "Ctrl+C to quit\n";
}

void constrain(float& x, float min, float max) {
    x = (x < min ? min : (x > max ? max : x));
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
    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate rate(50.);
    // Другие параметры
	bool end = false;
    MobileParam param_control,
    			param_twist;
    
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

    help_param(argv[0]);
    help();

    float wheelbase = (argc - 1 > 1 ? argv[1] ? 1.0);
    	  max_backward_speed = (argc - 2 > 1 ? argv[2] ? 0.5),
    	  max_forward_speed = (argc - 3 > 1 ? argv[3] ? 1.0),
    	  steering_range = (argc - 4 > 1 ? argv[4] ? 30.0);
    
    while(ros::ok() && !end) {
        if(is_readable()) {
        	char key = getch();
        	switch(key) {
            	case ' ':
            	case 'k':
                	param_control.speed = 0.0;
                	param_control.turn = 0.0;
            	break;
                
            	case 'w':
                	param_control.speed += 0.05;
            	break;
            
            	case 's':
                	param_control.speed -= 0.05;
            	break;
            
            	case 'a':
                	param_control.turn += 1;
           		break;
            
            	case 'd':
                	param_control.turn -= 1;
            	break;
            	case '\x003':
                	end = true;
                	return 0;
            	break;
        	}
        	constrain(param_control.speed, -max_backward_speed, max_forward_speed);
        	constrain(param_control.turn, 0.0 - steering_range, 0.0 + steering_range);

        	param_twist.speed = param_control.speed;
        	param_twist.turn = param_twist.speed / (wheelbase / std::tan(param_control.turn));
        }

    	geometry_msgs::Twist cmd;
    	cmd.linear.x = param_twist.speed; cmd.linear.y = 0.0; cmd.linear.z = 0.0;
    	cmd.angular.x = 0.0; cmd.angular.y = 0.0; cmd.angular.z = param_twist.turn;

		publisher.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
