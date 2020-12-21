#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#define PI 				3.14159265358979

class Hardware {
private:
	ros::Subscriber sub_vel, sub_enc;
	ros::Publisher pub_odom, pub_vel;
	ros::Timer timer_1, timer_2;

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

	double radian(int angle);
	double degree(double radian);
public:
	Hardware(ros::NodeHandle nh_);

	void velCB(const geometry_msgs::Twist::ConstPtr& cmd_);
	void speedCB(const std_msgs::Float64::ConstPtr& speed_);
	void goCB(const ros::TimerEvent& timer);
	void odomCB(const ros::TimerEvent& timer);
};

Hardware::Hardware(ros::NodeHandle nh_) {
	sub_vel = nh_.subscribe("cmd_vel", 25, &Hardware::velCB, this);
	sub_enc = nh_.subscribe("/mobile/speed", 25, &Hardware::speedCB, this);
	pub_vel = nh_.advertise<std_msgs::Float64MultiArray>("/mobile/cmd_vel", 25);
	pub_odom = nh_.advertise<nav_msgs::Odometry>("odom", 25);

	timer_1 = nh_.createTimer(ros::Duration((1.0) / 10), &Hardware::odomCB, this);
	timer_2 = nh_.createTimer(ros::Duration((1.0) / 25), &Hardware::goCB, this);

	cmd_akk.data.push_back(0);
	cmd_akk.data.push_back(90);
	
	speed.data = 0.0;
}

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

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_hardware");
	ros::NodeHandle nh;
	Hardware hardware(nh);
	ros::spin();
}
