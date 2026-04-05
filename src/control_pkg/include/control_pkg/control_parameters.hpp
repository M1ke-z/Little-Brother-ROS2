#ifndef CONTROL_PARAMETERS
#define CONTROL_PARAMETERS

#include "rclcpp/rclcpp.hpp"
#include <array>

/*      Declare Const Variables     */ 
constexpr int NUM_MOTORS = 16;
constexpr int DEFAULT_MAX_US = 2400;
constexpr int DEFAULT_MIN_US = 544;

/*      Declare Structs     */ 
struct MotorData {
    double max_angle;
    double min_angle;
    double offset;
    double min_us;
    double max_us;
    bool inverted;
};

/*      Declare Functions     */ 
void declare_motor_parameters(rclcpp::Node& node);
std::array<MotorData, NUM_MOTORS> get_motor_parameters(rclcpp::Node& node);

#endif