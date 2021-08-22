# mpu6050_ros

ROS node for MPU6050.

The library is written by referring to I2Cdev library code written by Jeff Rowberg [https://github.com/jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib).

> Notes: The library does not provide communication with all regiters on MPU6050 yet.
However, basic usages and DMP feature are already supported.

## Dependencies
- wiringPi
<br>The driver mainly uses GPIO to control the motor. For using c++ to control the GPIO, wiringPi is needed.
```
sudo apt-get install wiringpi
```
