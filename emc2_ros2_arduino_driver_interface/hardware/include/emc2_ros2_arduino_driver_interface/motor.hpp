#ifndef EMC2_ROS2_ARDUINO_DRIVER_INTERFACE_WHEEL_HPP
#define EMC2_ROS2_ARDUINO_DRIVER_INTERFACE_WHEEL_HPP

#include <string>
#include <cmath>


class Motor
{
    public:

    std::string name = "";
    double cmdAngVel = 0.0;
    double angPos = 0.0;
    double angVel = 0.0;

    Motor() = default;

    Motor(const std::string &motor_wheel_name)
    {
      setup(motor_wheel_name);
    }

    
    void setup(const std::string &motor_wheel_name)
    {
      name = motor_wheel_name;
    }



};


#endif // EMC2_ROS2_ARDUINO_DRIVER_INTERFACE_WHEEL_HPP
