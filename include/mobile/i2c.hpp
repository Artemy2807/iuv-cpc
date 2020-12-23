#pragma once
#include <linux/i2c-dev.h> 
#include <string>

//
#define ROS_
#ifdef ROS_
    #include <ros/ros.h>
    #define MSG(x) \
        ROS_INFO("%s", x)
#else
    #include <iostream>
    #define MSG(x) \
        std::cout << "[INFO]: " << x << std::endl; 
#endif

namespace i2c {

    struct Device {
        int number_i2c,
            address;
        char filename[20];

        Device(int number_i2c_, int address_) :
            number_i2c(number_i2c_),
            address(address_)
        {
            snprintf(filename, 19, "/dev/i2c-%d", number_i2c);
        }
    }

    class I2C {
    public:
        I2C() = default;
        I2C(const Device& device_);

        bool open(const Device& device_);

        ssize_t read(void* receive, size_t size);
        ssize_t write(void* transmitter, size_t size);
    private:
        int* fd;

        Device device;
    };

}
