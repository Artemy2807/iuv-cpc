#include "i2c.hpp"

namespace i2c {
    I2C::I2C(const Device& device_) {
        open(device_);
    }

    bool I2C::open(const Device& device_) {
        device = device_;


        if((fd = ::open(device.filename, O_RDWR)) < 0) {
            MSG("can not access to the adapter");
            return false;
        }

        if(ioctl(fd, I2C_SLAVE, device.address) < 0) {
            MSG("can not opened the device");
            return false;
        }

        return true;
    }

    ssize_t I2C::read(void* receive, size_t size) {
        ssize_t size_ = 0;

        if((size_ = ::read(fd, receive, size)) < 0) {
            MSG("can not read data from the device");
        }

        return size_;
    }

    ssize_t I2C::write(void* transmitter, size_t size) {
        ssize_t size_ = 0;

        if((size_ = ::write(fd, transmitter, size)) < 0) {
            MSG("can not send data to the device");
        }

        return size_;
    }
}
