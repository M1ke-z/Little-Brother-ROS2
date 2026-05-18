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

/* Declare Classes */

class gait{
    public:
        gait();

        gait(
            std::vector<int64_t> phase_offsets, 
            double minFrequency,
            double maxFrequency, 
            std::vector<std::vector<double>> swing_control_points, 
            std::vector<std::vector<double>> stance_control_points, 
            std::vector<double> curve_offsets, 
            double swing_switch_phase,
            std::vector<double> center_point
        );

        double getMinGaitFrequency() const;

        double getMaxGaitFrequency() const;

        std::vector<int64_t> getPhaseOffset() const;

        std::vector<double> getCurveOffsets() const;

        std::vector<std::vector<double>> getSwingControlPoints() const;

        std::vector<std::vector<double>> getStanceControlPoints() const;

        double getSwingSwitchPhase() const;

        std::vector<double> getCenterPoint() const;

    private:
        std::vector<int64_t> phaseOffsets;
        double minGaitFrequency;
        double maxGaitFrequency;

        std::vector<double> curveCenterPoint;

        std::vector<double> curveOffsets;

        std::vector<std::vector<double>> swingControlPoints; // Length is 6 by default (Don't change until bernstein polynomials are implemented for general n)
        std::vector<std::vector<double>> stanceControlPoints; // Length is 2 by default (Don't change for now bernstein polynomials are implemented for general n)

        double swingSwitchPhase;
};

gait get_gait_params(rclcpp::Node& node, std::string gait_name);

#endif