#include "control_pkg/control_parameters.hpp"
#include <cmath>

void declare_motor_parameters(rclcpp::Node& node)
{
    /* ------
        Purpose:
            Declares all of the motor parameters before they are accessed
        Parameters:
            rclcpp::Node& node - The node that the parameters should be declared in
        Return 
            None
        ------ */ 

    // For each motor define the basic properties associated with it
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
    /* ------
        Purpose:
            After the motor parameters have been declared this function can be called to return an array of the values for 
            each motor
        Parameters:
            rclcpp::Node& node - The node that the parameters belong to 
        Return 
            std::array<MotorData, NUM_MOTORS> - An array with all of the motor values
        ------ */ 

    std::array<MotorData, NUM_MOTORS> motorData;

    // Get the motor data for each motor
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
    /* ------
        Purpose:
            This function will return the gait params for a requested gait. 
            This needs to be done on a per gait basis since there is not a fixed number of gaits
        Parameters:
            rclcpp::Node& node - The node that the parameters belong to 
            std::string gaitName - The name of the gait that we want the parameters for
        Return 
            gait - A gait object containing all of the parameters for a requested gait
        ------ */ 

    // Declare variables to temporarly hold all of the gate parameters
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

    // Declare the parameters within the node provided
    node.declare_parameter<std::vector<int64_t>>(gaitName + ".phaseOffsets");
    node.declare_parameter<double>(gaitName + ".minFrequency");
    node.declare_parameter<double>(gaitName + ".maxFrequency");
    node.declare_parameter<std::vector<double>>(gaitName + ".swingControlPoints");
    node.declare_parameter<std::vector<double>>(gaitName + ".stanceControlPoints");
    node.declare_parameter<std::vector<double>>(gaitName + ".curveOffsets");
    node.declare_parameter<double>(gaitName + ".swingSwitchPhase");
    node.declare_parameter<std::vector<double>>(gaitName + ".centerPoint");

    // Get the parameters and store them in the designated variable
    node.get_parameter(gaitName + ".phaseOffsets", phaseOffsets);
    node.get_parameter(gaitName + ".minFrequency", minFrequency);
    node.get_parameter(gaitName + ".maxFrequency", maxFrequency);
    node.get_parameter(gaitName + ".swingControlPoints", flatSwingControlPoints);
    node.get_parameter(gaitName + ".stanceControlPoints", flatStanceControlPoints);
    node.get_parameter(gaitName + ".curveOffsets", curveOffsets);
    node.get_parameter(gaitName + ".swingSwitchPhase", swingSwitchPhase);
    node.get_parameter(gaitName + ".centerPoint", centerPoint);

    // Both of these for loops convert the flat array of control points into a 2D array.
    // This is required since ROS2 does not allow you to store multi-dimensional arrays as a parameter file
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

    // Generate the gait and return it
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