#include <ros/ros.h>
#include <angles/angles.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "i2c.hpp"

class Hardware : public hardware_interface::RobotHW {
public:
	Hardware(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer);

private:
    
    enum class JointType {
        NONE = 0,
        MOTOR,
        SERVO,
    };
    
	ros::Timer timer_1;
	ros::Duration elapsed_time;
    int loop_hz = 0;
    
    std::shared_ptr<controller_manager::ControllerManager> controller_manager;

	i2c::I2C i2c_device;
    
    hardware_interface::JointStateInterface joint_state_interface;
	hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
	
	double joint_position[2];
    double joint_velocity[2];
    double joint_effort[2];
    double joint_cmd[2];
    double wheel_radians = 0.0;
	int position_prev = 0,
		velocity_prev = 0;

	ros::Time time_prev;
    
    void init(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer);
    void update(const ros::TimerEvent& e);
	void read();
	void write();
};

Hardware::Hardware(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer) {
    // Подключение к arduino по i2c
	i2c::Device device_des(0, 0x08);
	i2c_device.open(device_des);
	
	init(nh_, rear_wheel, front_steer);

    controller_manager.reset(new controller_manager::ControllerManager(this, nh_));
    
    loop_hz = 10;
    timer_1 = nh_.createTimer(ros::Duration(1.0 / loop_hz), &Hardware::update, this);

	time_prev = ros::Time::now();
}

void Hardware::init(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer) {
	std::string names[2] = { rear_wheel, front_steer };
	for(int i = 0; i < 2; i++) {
		hardware_interface::JointStateHandle joint_state_handle(names[i], 
                                            &joint_position[i], &joint_velocity[i], &joint_effort[i]);
		joint_state_interface.registerHandle(joint_state_handle);
	}

	hardware_interface::JointHandle joint_state_handle_velocity(joint_state_interface.getHandle(rear_wheel),
														&joint_cmd[0]);
	velocity_joint_interface.registerHandle(joint_state_handle_velocity);

	hardware_interface::JointHandle joint_state_handle_position(joint_state_interface.getHandle(front_steer), 
                                                        &joint_cmd[1]);
	position_joint_interface.registerHandle(joint_state_handle_position);

	/*
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
	*/

    
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
    //registerInterface(&velocity_jount_saturation);
    registerInterface(&position_joint_interface);
    //registerInterface(&position_jount_saturation);
}

void Hardware::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);
	ROS_INFO("%f", (e.current_real - e.last_real).toSec());
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write();
}

void Hardware::read() {
	// Считывание количества градусов, которое проехала модель
	int32_t data;
	i2c_device.read((void*)&data, 4);

	// Преобразование градусов в радианы
	wheel_radians += angles::from_degrees((double)data);
	joint_position[0] = wheel_radians;
	//joint_position[1] = angles::from_degrees(position_prev);
	ROS_INFO("Joint command velocity: %f", angles::to_degrees(joint_position[0]));
}

void Hardware::write() {
	// velocity - сколько градусов необходимо проехать
	// position - угол поворота передних колёс в градусах
	// Так как в ros используются система единиц СИ, необходимо 
	// радианы преобразовать в градусы
	int velocity = (int)angles::to_degrees(joint_cmd[0]),
		position = (int)angles::to_degrees(joint_cmd[1]);

	ROS_INFO("\n\tVelocity: %d\n\tPosition: %d", velocity, position);
	//position = 20;

	ros::Time time_current = ros::Time::now();
	
	if((velocity_prev != velocity) || (position_prev != position) || (time_current - time_prev).toSec() > 0.5) {
		// Создание отправляемых данных
		// Все данные записываются в wbuf:
		// [1-2 байты] - сколько градусов необходимо проехать (выделяется 2 байта,
		// так как значение градусов может превышать 255 градусов)
		// [3 байт] - угол поворота передних колёс
		uint8_t wbuf[3];
		std::memcpy(&wbuf[0], &velocity, 2);
		wbuf[2] = 90 + ((int)position * 2);

		// Отправление данных на arduino
		i2c_device.write((void*)&wbuf, 3);
		
		position_prev = position;
		velocity_prev = velocity;
		time_prev = time_current;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_hardware");
	ros::NodeHandle nh;
	ros::MultiThreadedSpinner spinner(2);
	Hardware hardware(nh,
					"rear_wheel_joint", 		// Название узла заднего колеса	
					"front_steer_joint");		// Название узла переднего рулевого управления
	spinner.spin();
	return 0;
}
