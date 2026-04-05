#include "control_pkg/control_parameters.hpp"
#include <cmath>

void declare_motor_parameters(rclcpp::Node& node)
{
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        node.declare_parameter<double>("motor" + std::to_string(i) + ".max_angle", M_PI_2);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".min_angle", -M_PI_2);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".max_us", DEFAULT_MAX_US);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".min_us", DEFAULT_MIN_US);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".offset", 0);
        node.declare_parameter<bool>("motor" + std::to_string(i) + ".inverted", false);
    }
    return;
}

std::array<MotorData, NUM_MOTORS> get_motor_parameters(rclcpp::Node& node) {
    std::array<MotorData, NUM_MOTORS> motor_data;
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        node.get_parameter("motor" + std::to_string(i) + ".max_angle", motor_data[i].max_angle);
        node.get_parameter("motor" + std::to_string(i) + ".min_angle", motor_data[i].min_angle);
        node.get_parameter("motor" + std::to_string(i) + ".max_us", motor_data[i].max_us);
        node.get_parameter("motor" + std::to_string(i) + ".min_us", motor_data[i].min_us);
        node.get_parameter("motor" + std::to_string(i) + ".offset", motor_data[i].offset);
        node.get_parameter("motor" + std::to_string(i) + ".inverted", motor_data[i].inverted);
    }
    return motor_data;
}