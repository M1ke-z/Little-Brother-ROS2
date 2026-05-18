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
    double maxAngle;
    double minAngle;
    double offset;
    double minUs;
    double maxUs;
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
            std::vector<int64_t> phaseOffsets, 
            double minFrequency,
            double maxFrequency, 
            std::vector<std::vector<double>> swingControlPoints, 
            std::vector<std::vector<double>> stanceControlPoints, 
            std::vector<double> curveOffsets, 
            double swingSwitchPhase,
            std::vector<double> centerPoint
        );

        double get_min_gait_frequency() const;

        double get_max_gait_frequency() const;

        std::vector<int64_t> get_phase_offset() const;

        std::vector<double> get_curve_offsets() const;

        std::vector<std::vector<double>> get_swing_control_points() const;

        std::vector<std::vector<double>> get_stance_control_points() const;

        double get_swing_switch_phase() const;

        std::vector<double> get_center_point() const;

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

gait get_gait_params(rclcpp::Node& node, std::string gaitName);

#endif