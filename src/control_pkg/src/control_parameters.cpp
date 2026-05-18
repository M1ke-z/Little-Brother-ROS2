#include "control_pkg/control_parameters.hpp"
#include <cmath>

void declare_motor_parameters(rclcpp::Node& node)
{
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        node.declare_parameter<double>("motor" + std::to_string(i) + ".maxAngle", M_PI_2);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".minAngle", -M_PI_2);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".maxUs", DEFAULT_MAX_US);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".minUs", DEFAULT_MIN_US);
        node.declare_parameter<double>("motor" + std::to_string(i) + ".offset", 0);
        node.declare_parameter<bool>("motor" + std::to_string(i) + ".inverted", false);
    }
    return;
}

std::array<MotorData, NUM_MOTORS> get_motor_parameters(rclcpp::Node& node) {
    std::array<MotorData, NUM_MOTORS> motorData;
    for(uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        node.get_parameter("motor" + std::to_string(i) + ".maxAngle", motorData[i].maxAngle);
        node.get_parameter("motor" + std::to_string(i) + ".minAngle", motorData[i].minAngle);
        node.get_parameter("motor" + std::to_string(i) + ".maxUs", motorData[i].maxUs);
        node.get_parameter("motor" + std::to_string(i) + ".minUs", motorData[i].minUs);
        node.get_parameter("motor" + std::to_string(i) + ".offset", motorData[i].offset);
        node.get_parameter("motor" + std::to_string(i) + ".inverted", motorData[i].inverted);
    }
    return motorData;
}

// Gait class function definitions

gait::gait() = default;

gait::gait(
    std::vector<int64_t> phaseOffsets, 
    double minFrequency,
    double maxFrequency, 
    std::vector<std::vector<double>> swingControlPoints, 
    std::vector<std::vector<double>> stanceControlPoints, 
    std::vector<double> curveOffsets, 
    double swingSwitchPhase,
    std::vector<double> centerPoint):
    phaseOffsets(phaseOffsets),
    minGaitFrequency(minFrequency),
    maxGaitFrequency(maxFrequency),
    swingControlPoints(swingControlPoints),
    stanceControlPoints(stanceControlPoints),
    curveOffsets(curveOffsets),
    swingSwitchPhase(swingSwitchPhase),
    curveCenterPoint(centerPoint){}

double gait::get_min_gait_frequency() const { return minGaitFrequency; }

double gait::get_max_gait_frequency() const { return maxGaitFrequency; }

std::vector<int64_t> gait::get_phase_offset() const { return phaseOffsets; }

std::vector<double> gait::get_curve_offsets() const { return curveOffsets; }

std::vector<std::vector<double>> gait::get_swing_control_points() const { return swingControlPoints; }

std::vector<std::vector<double>> gait::get_stance_control_points() const { return stanceControlPoints; }

double gait::get_swing_switch_phase() const { return swingSwitchPhase; }

std::vector<double> gait::get_center_point() const { return curveCenterPoint; }


gait get_gait_params(rclcpp::Node& node, std::string gaitName) {
    std::vector<int64_t> phaseOffsets;
    double minFrequency;
    double maxFrequency;
    std::vector<double> flatSwingControlPoints;
    std::vector<std::vector<double>> swingControlPoints;
    std::vector<double> flatStanceControlPoints;
    std::vector<std::vector<double>> stanceControlPoints;
    std::vector<double> curveOffsets;
    double swingSwitchPhase;
    std::vector<double> centerPoint;

    node.declare_parameter<std::vector<int64_t>>(gaitName + ".phaseOffsets");
    node.declare_parameter<double>(gaitName + ".minFrequency");
    node.declare_parameter<double>(gaitName + ".maxFrequency");
    node.declare_parameter<std::vector<double>>(gaitName + ".swingControlPoints");
    node.declare_parameter<std::vector<double>>(gaitName + ".stanceControlPoints");
    node.declare_parameter<std::vector<double>>(gaitName + ".curveOffsets");
    node.declare_parameter<double>(gaitName + ".swingSwitchPhase");
    node.declare_parameter<std::vector<double>>(gaitName + ".centerPoint");

    node.get_parameter(gaitName + ".phaseOffsets", phaseOffsets);
    node.get_parameter(gaitName + ".minFrequency", minFrequency);
    node.get_parameter(gaitName + ".maxFrequency", maxFrequency);
    node.get_parameter(gaitName + ".swingControlPoints", flatSwingControlPoints);
    node.get_parameter(gaitName + ".stanceControlPoints", flatStanceControlPoints);
    node.get_parameter(gaitName + ".curveOffsets", curveOffsets);
    node.get_parameter(gaitName + ".swingSwitchPhase", swingSwitchPhase);
    node.get_parameter(gaitName + ".centerPoint", centerPoint);

    for(size_t index = 0; index < flatStanceControlPoints.size(); index+=3){
        stanceControlPoints.push_back(std::vector<double> {
            flatStanceControlPoints[index], 
            flatStanceControlPoints[index+1], 
            flatStanceControlPoints[index+2]
        });
    } 

    for(size_t index = 0; index < flatSwingControlPoints.size(); index+=3){
        swingControlPoints.push_back(std::vector<double> {
            flatSwingControlPoints[index], 
            flatSwingControlPoints[index+1], 
            flatSwingControlPoints[index+2]
        });
    } 

    gait returnGait(
        phaseOffsets,
        minFrequency,
        maxFrequency,
        swingControlPoints,
        stanceControlPoints,
        curveOffsets,
        swingSwitchPhase,
        centerPoint
    );

    return returnGait;
}