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

// Gait class function definitions

gait::gait() = default;

gait::gait(
    std::vector<int64_t> phase_offsets, 
    double minFrequency,
    double maxFrequency, 
    std::vector<std::vector<double>> swing_control_points, 
    std::vector<std::vector<double>> stance_control_points, 
    std::vector<double> curve_offsets, 
    double swing_switch_phase,
    std::vector<double> center_point):
    phaseOffsets(phase_offsets),
    minGaitFrequency(minFrequency),
    maxGaitFrequency(maxFrequency),
    swingControlPoints(swing_control_points),
    stanceControlPoints(stance_control_points),
    curveOffsets(curve_offsets),
    swingSwitchPhase(swing_switch_phase),
    curveCenterPoint(center_point){}

double gait::getMinGaitFrequency() const { return minGaitFrequency; }

double gait::getMaxGaitFrequency() const { return maxGaitFrequency; }

std::vector<int64_t> gait::getPhaseOffset() const { return phaseOffsets; }

std::vector<double> gait::getCurveOffsets() const { return curveOffsets; }

std::vector<std::vector<double>> gait::getSwingControlPoints() const { return swingControlPoints; }

std::vector<std::vector<double>> gait::getStanceControlPoints() const { return stanceControlPoints; }

double gait::getSwingSwitchPhase() const { return swingSwitchPhase; }

std::vector<double> gait::getCenterPoint() const { return curveCenterPoint; }


gait get_gait_params(rclcpp::Node& node, std::string gait_name) {
    std::vector<int64_t> phase_offsets;
    double min_frequency;
    double max_frequency;
    std::vector<double> flat_swing_control_points;
    std::vector<std::vector<double>> swing_control_points;
    std::vector<double> flat_stance_control_points;
    std::vector<std::vector<double>> stance_control_points;
    std::vector<double> curve_offsets;
    double swing_switch_phase;
    std::vector<double> center_point;

    node.declare_parameter<std::vector<int64_t>>(gait_name + ".phase_offsets");
    node.declare_parameter<double>(gait_name + ".min_frequency");
    node.declare_parameter<double>(gait_name + ".max_frequency");
    node.declare_parameter<std::vector<double>>(gait_name + ".swing_control_points");
    node.declare_parameter<std::vector<double>>(gait_name + ".stance_control_points");
    node.declare_parameter<std::vector<double>>(gait_name + ".curve_offsets");
    node.declare_parameter<double>(gait_name + ".swing_switch_phase");
    node.declare_parameter<std::vector<double>>(gait_name + ".center_point");

    node.get_parameter(gait_name + ".phase_offsets", phase_offsets);
    node.get_parameter(gait_name + ".min_frequency", min_frequency);
    node.get_parameter(gait_name + ".max_frequency", max_frequency);
    node.get_parameter(gait_name + ".swing_control_points", flat_swing_control_points);
    node.get_parameter(gait_name + ".stance_control_points", flat_stance_control_points);
    node.get_parameter(gait_name + ".curve_offsets", curve_offsets);
    node.get_parameter(gait_name + ".swing_switch_phase", swing_switch_phase);
    node.get_parameter(gait_name + ".center_point", center_point);

    for(size_t index = 0; index < flat_stance_control_points.size(); index+=3){
        stance_control_points.push_back(std::vector<double> {
            flat_stance_control_points[index], 
            flat_stance_control_points[index+1], 
            flat_stance_control_points[index+2]
        });
    } 

    for(size_t index = 0; index < flat_swing_control_points.size(); index+=3){
        swing_control_points.push_back(std::vector<double> {
            flat_swing_control_points[index], 
            flat_swing_control_points[index+1], 
            flat_swing_control_points[index+2]
        });
    } 

    gait return_gait(
        phase_offsets,
        min_frequency,
        max_frequency,
        swing_control_points,
        stance_control_points,
        curve_offsets,
        swing_switch_phase,
        center_point
    );

    return return_gait;
}