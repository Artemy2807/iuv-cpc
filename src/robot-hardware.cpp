#include <ros/ros.h>
#include <angles/angles.h>
#include "i2c.hpp"

class Hardware : public hardware_interface::RobotHW {
public:
	Hardware(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer);

private:
    
    struct class JointType {
        NONE = 0,
        MOTOR,
        SERVO,
    };
    
	ros::Timer timer_1;
    int loop_hz = 0;
    
    std::shared_ptr<controller_manager::ControllerManager> controller_manager;

	i2c::I2C i2c_device;
    
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
	
	double joint_position[2];
    double joint_velocity[2];
    double joint_effort[2];
    double joint_cmd,
    	wheel_radians = 0.0,
	int position_prev = 0,
		velocity_prev = 0;
    
    void init(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer);
    void update(const ros::TimerEvent& e);
	void read();
	void write();
};

Hardware::Hardware(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer) {
    // Подключение к arduino по i2c
	i2c::Device device_des(1, 0x10);
	i2c_device.open(device_des);
	
	init(nh_);
    
    controller_manager.reset(new controller_manager::ControllerManager(this, nh_));
    
    loop_hz = 10;
    timer_1 = nh_.createTimer(ros::Duration(1.0 / loop_hz), &Hardware::update, this);
}

void Hardware::init(ros::NodeHandle& nh_, std::string rear_wheel, std::string front_steer) {
	std::string names[2] = { rear_wheel, front_steer };
	for(int i = 0; i < 2; i++) {
		hardware_interface::JointStateHandle joint_state_handle(&names[i], 
                                            &joint_position[i], &joint_velocity[i], &joint_effort[i]);
		joint_state_interface.registerHandle(joint_state_handle);
	}

	hardware_interface::JointHandle joint_state_handle(joint_state_interface.getHandle(rear_wheel), 
                                                        &joint_cmd);
	position_joint_interface.registerHandle(joint_state_handle);

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
    //registerInterface(&velocity_joint_interface);
    //registerInterface(&velocity_jount_saturation);
    registerInterface(&position_joint_interface);
    //registerInterface(&position_jount_saturation);
}

void Hardware::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write();
}

void Hardware::read() {
	// Считывание количества градусов, которое проехала модель
	uint8_t value;
	i2c_device.read((char*)&value, 1);

	// Преобразование градусов в радианы
	wheel_radians += angles::from_degrees((double)value);
	joint_cmd = wheel_radians;
}

void Hardware::write() {
	// velocity - сколько градусов необходимо проехать
	// position - угол поворота передних колёс в градусах
	// Так как в ros используются система единиц СИ, необходимо 
	// радианы преобразовать в градусы
	int velocity = (int)angles::to_degrees(joint_velocity[0]),
		position = (int)angles::to_degrees(joint_position[1]);
	
	if((velocity_prev != velocity) || (position_prev != position)) {
		// Создание отправляемых данных
		// Все данные записываются в wbuf:
		// [1-2 байты] - сколько градусов необходимо проехать (выделяется 2 байта,
		// так как значение градусов может превышать 255 градусов)
		// [3 байт] - угол поворота передних колёс
		uint8_t wbuf[3];
		std::memcpy(&wbuf[0], velocity, 2);
		wbuf[2] = position;

		// Отправление данных на arduino
		i2c_device.write((void*)&wbuf, 3);
		
		position_prev = position;
		velocity_prev = velocity;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_hardware");
	ros::NodeHandle nh;
	Hardware hardware(nh,
					"rear_wheel_joint", 		// Название узла заднего колеса	
					"front_steer_joint");		// Название узла переднего рулевого управления
	ros::spin();
}
