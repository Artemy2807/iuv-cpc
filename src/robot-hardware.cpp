// TODO: Почистить код

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#define PI 				3.14159265358979

class Hardware : public hardware_interface::RobotHW {
private:
    
    struct class JointType {
        NONE = 0,
        MOTOR,
        SERVO,
    };
    
	ros::Subscriber sub_vel, sub_enc;
	ros::Publisher pub_odom, pub_vel;
	ros::Timer timer_1, timer_2;
    int loop_hz = 0;
    
    std::shared_ptr<controller_manager::ControllerManager> controller_manager;

/*
    
	geometry_msgs::Twist cmd_vel;
	std_msgs::Float64MultiArray cmd_akk;
	std_msgs::Float64 speed;

	double steer_angle = 0.0,
		propel = 0.0,
		dt = 0.0,
		dth = 0.0,
		dx = 0.0,
		dy = 0.0,
		x = 0.0,
		y = 0.0,
		th = 0.0;
	ros::Time current_time;
	tf::TransformBroadcaster odom_broadcaster;
    
*/
    
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    
    joint_limits_interface::VelocityJointSaturationInterface velocity_jount_saturation;
    joint_limits_interface::PositionJointSaturationInterface position_jount_saturation;
    
    std::vector<std::pair<std::string, JointType>> joint_name = 
                    { std::make_pair("", JointType::MOTOR), std::make_pair("", JointType::SERVO) };
    double joint_position[2];
    double joint_velocity[2];
    double joint_effort[2];
    double joint_cmd[2];
    
/*

	double radian(int angle);
	double degree(double radian);
	
*/
    
    void init(ros::NodeHandle& nh_);
    void update(const ros::TimerEvent& e);
public:
	Hardware(ros::NodeHandle& nh_);

/*

	void velCB(const geometry_msgs::Twist::ConstPtr& cmd_);
	void speedCB(const std_msgs::Float64::ConstPtr& speed_);
	void goCB(const ros::TimerEvent& timer);
	void odomCB(const ros::TimerEvent& timer);
	
*/

};

Hardware::Hardware(ros::NodeHandle& nh_) {
    init(nh_);
    
    controller_manager.reset(new controller_manager::ControllerManager(this, nh_));
    
    loop_hz = 10;
    timer_1 = nh_.createTimer(ros::Duration(1.0 / loop_hz), &Hardware::update, this);
    
/*
	sub_vel = nh_.subscribe("cmd_vel", 25, &Hardware::velCB, this);
	sub_enc = nh_.subscribe("/mobile/speed", 25, &Hardware::speedCB, this);
	pub_vel = nh_.advertise<std_msgs::Float64MultiArray>("/mobile/cmd_vel", 25);
	pub_odom = nh_.advertise<nav_msgs::Odometry>("odom", 25);

	timer_1 = nh_.createTimer(ros::Duration((1.0) / 10), &Hardware::odomCB, this);
	timer_2 = nh_.createTimer(ros::Duration((1.0) / 25), &Hardware::goCB, this);

	cmd_akk.data.push_back(0);
	cmd_akk.data.push_back(90);
	
	speed.data = 0.0;
*/

}

void Hardware::init(ros::NodeHandle& nh_) {
    for(int i = 0; i < 2; i++) {
        hardware_interface::JointStateHandle joint_state_handle(&joint_name[i].first, 
                                            &joint_position[i], &joint_velocity[i], &joint_effort[i]);
        joint_state_interface.registerHandle(joint_state_handle);
        
        hardware_interface::JointHandle joint_state_handle(joint_state_handle, 
                                                           &joint_cmd[i]);
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(&joint_name[i].first, nh_, limits);
        
        if(joint_name[i].second == JointType::MOTOR) {
            joint_limits_interface::VelocityJointSaturationHandle joint_limits_handle(joint_state_handle, limits);
            
            velocity_joint_interface.registerHandle(joint_state_handle);
            velocity_jount_saturation.registerHandle(joint_limits_handle);
        }else {
            joint_limits_interface::PositionJointSaturationHandle joint_limits_handle(joint_state_handle, limits);
            
            position_joint_interface.registerHandle(joint_state_handle);
            position_jount_saturation.registerHandle(joint_limits_handle);
        }
    }
    
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
    registerInterface(&velocity_jount_saturation);
    registerInterface(&position_joint_interface);
    registerInterface(&position_jount_saturation);
}

void Hardware::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}

/*

double Hardware::radian(int angle) {
	return double(angle) * (PI / 180.0);
}

double Hardware::degree(double radian) {
	return double(radian) * (180.0 / PI);
}

void Hardware::velCB(const geometry_msgs::Twist::ConstPtr& cmd_) {
	cmd_vel = *cmd_;
	//ROS_INFO("/n/tspeed: %d, turn: %d", (int)cmd_vel.data[0], (int)cmd_vel.data[1]);
}

void Hardware::speedCB(const std_msgs::Float64::ConstPtr& speed_) {
	speed = *speed_;
}

void Hardware::goCB(const ros::TimerEvent& timer) {
	cmd_akk.data[0] = cmd_vel.linear.x;
	if(cmd_vel.linear.x != 0.0 && cmd_vel.angular.z != 0.0) {
		double radius = cmd_vel.linear.x / cmd_vel.angular.z;
		cmd_akk.data[1] = degree(atan(0.32 / radius));
		cmd_akk.data[1] += 90.0;
	}
	pub_vel.publish(cmd_akk);
	ROS_INFO("\n\tspeed x: %f, angle: %f, servo_angle: %f", cmd_vel.linear.x, cmd_vel.angular.z, cmd_akk.data[1]);
}

void Hardware::odomCB(const ros::TimerEvent& timer) {
	steer_angle = radian(cmd_akk.data[1] - 90.0);
	propel = (double)speed.data;
	dt = (timer.current_real - timer.last_real).toSec();
	dth = propel * std::tan(steer_angle) / 0.32;
	dx = propel * std::cos(th + ((dth * dt) / 2.0));
	dy = propel * std::sin(th + ((dth * dt) / 2.0));

	x += dx * dt;
	y += dy * dt;
	th += dth * dt;
	ROS_INFO("\n\tspeed: %f, steer_angle: %f, x: %f, y: %f, th: %f", propel, (double)steer_angle,(double) x, (double)y, (double)th);
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = timer.current_real;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = timer.current_real;
	odom.header.frame_id = "odom";

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = dx;
	odom.twist.twist.linear.y = dy;
	odom.twist.twist.angular.z = dth;

	pub_odom.publish(odom);
}

*/

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_hardware");
	ros::NodeHandle nh;
	Hardware hardware(nh);
	ros::spin();
}
