#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "control_pkg/control_parameters.hpp"

#include "interfaces/msg/int.hpp"
#include "interfaces/msg/double.hpp"
#include "interfaces/msg/motor_position.hpp"
#include "interfaces/msg/motor_testing.hpp"
#include "interfaces/msg/balance_correction.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "sensor_msgs/msg/joy.hpp"

// class for containing all functions and variables related to gait generation
class globalGaitControl {
    public:

        globalGaitControl() = default;

        void queue_gait(gait *gait){
            /* ------
                Purpose:
                    Sets a primary to transition gait based on the pointer passed into the function
                Parameters:
                    gait *gait - takes pointer to a gait object that should be queued as a gait
                Return 
                    void
               ------ */ 

            // If a primary gait doesn't exist then set the provided gait as primary
            if(primaryGait == nullptr) {
                primaryGait = gait;
            }
            // If a primary gait exists and there is no existing transition gait then set the passed value as the transition
            else if(transitionGait == nullptr){
                transitionGait = gait;
                inTransition = true;
            }
        }

        std::array<double, 3> get_foot_position(uint16_t legNum){
            /* ------
                Purpose:
                    Takes in a leg number and finds where the end effector should be based on the global phase clock
                Parameters:
                    uint16_t legNum - Provides which leg we want to find the foot position for
                Return 
                    std::array<double, 3> - Returns the x, y, z coordinates of the end effector
               ------ */ 

            // If there is no primary gait then return default coordinates instead. 
            // This should no longer be an issue since a default gait is being set
            if(primaryGait == nullptr) {
                return std::array<double, 3>{-20.0, -155.0, 0};
            }

            // Update leg turn scale based on the current gaitTurnScale and the active leg.  
            double legTurnScale = 0.0;

            if(gaitTurnScale < 0.0 && (legNum == 1 || legNum == 3)) legTurnScale = std::abs(gaitTurnScale);
            else if(gaitTurnScale > 0.0 && (legNum == 0 || legNum == 2)) legTurnScale = std::abs(gaitTurnScale);

            std::vector<int64_t> phaseOffsets = primaryGait->get_phase_offset();

            // calculate the primary phase based on the current global phase and the leg offset provided for each gait
            double primaryPhase = std::fmod(phaseClock + phaseOffsets[legNum], 360.0);
            double primarySwingSwitchPhase = primaryGait->get_swing_switch_phase();

            std::array<double, 3> primaryPosition;

            // If the leg is still in the swing movement then calculate the position based on the swing control points
            if(primaryPhase <= primarySwingSwitchPhase){
                primaryPosition = get_curve_points(
                    primaryGait->get_swing_control_points(), 
                    primaryGait->get_curve_offsets(), 
                    primaryPhase / primarySwingSwitchPhase,
                    legTurnScale,
                    primaryGait->get_center_point()
                );
            } 
            // If the leg has moved into the stance movement then calculate based on the stance control points and update the  
            // t value accordingly
            else{
                primaryPosition = get_curve_points(
                    primaryGait->get_stance_control_points(), 
                    primaryGait->get_curve_offsets(), 
                    (primaryPhase - primarySwingSwitchPhase) / (360.0 - primarySwingSwitchPhase),
                    legTurnScale,
                    primaryGait->get_center_point()
                );
            } 

            // If robot is transitioning to a new gait then we need to calulate the transition gait points and linearly
            // interpolate them
            if(inTransition){
                std::vector<int64_t> transitionPhaseOffsets = transitionGait->get_phase_offset();

                // calculate the transition phase based on the current global phase and the leg offset provided for each gait
                double transitionPhase = std::fmod(phaseClock + transitionPhaseOffsets[legNum], 360.0);
                double transitionSwingSwitchPhase = transitionGait->get_swing_switch_phase();

                std::array<double, 3> transitionPosition;
                
                // If the leg is still in the swing movement then calculate the position based on the swing control points
                if(transitionPhase <= transitionSwingSwitchPhase) transitionPosition = get_curve_points(
                    transitionGait->get_swing_control_points(), 
                    transitionGait->get_curve_offsets(), 
                    transitionPhase / transitionSwingSwitchPhase,
                    legTurnScale,
                    transitionGait->get_center_point()
                );
                // If the leg has moved into the stance movement then calculate based on the stance control points and update the  
                // t value accordingly
                else transitionPosition = get_curve_points(
                    transitionGait->get_stance_control_points(), 
                    transitionGait->get_curve_offsets(), 
                    (transitionPhase - transitionSwingSwitchPhase) / (360.0 - transitionSwingSwitchPhase),
                    legTurnScale,
                    transitionGait->get_center_point()
                );

                // Linearly interpolate between the points for each gait based on the current transition progress
                std::array<double, 3> blendedPosition = {
                    (1.0 - transitionProgress) * primaryPosition[0] + transitionProgress * transitionPosition[0],
                    (1.0 - transitionProgress) * primaryPosition[1] + transitionProgress * transitionPosition[1],
                    (1.0 - transitionProgress) * primaryPosition[2] + transitionProgress * transitionPosition[2]
                };
                
                // If balance offsets are enabled add the pitch and roll correction.
                // Note: This needs to be automatically updated based on what gait is running. For example having balance offsets
                //       enabled when running the pseduo trot gait makes it significantly worse
                if(enableBalanceOffsets)
                {
                    // Positive pitch correction should decrease the height of the front legs (y leg position is negative by default so 
                    // adding a pitch correction will decrease the height)
                    if(legNum == 0 || legNum == 1) blendedPosition[1] += pitchCorrection;
                    else blendedPosition[1] -= pitchCorrection;

                    // Positive roll correction will increase the height of the left side legs
                    if(legNum == 0 || legNum == 2) blendedPosition[1] -= rollCorrection;
                    else blendedPosition[1] += rollCorrection;
                }

                return blendedPosition;
            }
            else {
                // Duplicate of code from above
                // ToDo: Remove duplicate code by creating pitch and roll offsetion correction variables that can generically applied
                if(enableBalanceOffsets)
                {
                    if(legNum == 0 || legNum == 1) primaryPosition[1] += pitchCorrection;
                    else primaryPosition[1] -= pitchCorrection;

                    if(legNum == 0 || legNum == 2) primaryPosition[1] -= rollCorrection;
                    else primaryPosition[1] += rollCorrection;
                }

                return primaryPosition;
            }
        }

        std::array<std::array<double, 3>, 4> run_next_tick(){

            /* ------
                Purpose:
                    Calculates the next position for each leg based on primary and transition gaits and turning inputs
                Parameters:
                    None
                Return 
                    std::array<std::array<double, 3>, 4> - An array which contains four pairs of cartesian coordinates 
               ------ */ 

            // If the controller turn scale (provided by the controller) and gait turn scale (active turn scale used by global gait controller)
            // are not equal then increase the value based on the turn scale increment
            // Note: This should really factor in the time since the last tick since it is not guarenteed to run at 50Hz. 
            // Side Note: A microcontroller really should have been used for this project... If you are reading this, I made a dumb mistake...
            if(controllerTurnScale < gaitTurnScale) gaitTurnScale = std::max(gaitTurnScale - turnScaleIncrement, controllerTurnScale);
            else if(controllerTurnScale > gaitTurnScale) gaitTurnScale = std::min(gaitTurnScale + turnScaleIncrement, controllerTurnScale);

            double primaryGaitFrequency = 0.1;

            // If the primary gait exists then get the appropriate frequency percentage from the controller joystick and map it to the 
            // min and max frequency of the gait
            if(primaryGait != nullptr) {
                primaryGaitFrequency = primaryGait->get_min_gait_frequency() + (primaryGait->get_max_gait_frequency() - primaryGait->get_min_gait_frequency()) * frequencyScale;
            }

            if(inTransition){
                // Get the transition frequency based on the same logic above
                double transitionGaitFrequency = transitionGait->get_min_gait_frequency() + (transitionGait->get_max_gait_frequency() - transitionGait->get_min_gait_frequency()) * frequencyScale;

                // Increase the transition progress based on the gait frequency 
                // ToDo: 50.0 should really be a global variable
                transitionProgress += 1 / 50.0; // This should be based on the time since the last tick and the desired transition time
                transitionProgress = std::min(transitionProgress, 1.0);

                // If the transition has completed then set the primary gait to the transition gait and reset all of the other transition variables
                if(transitionProgress >= 1.0){
                    primaryGait = transitionGait;
                    transitionGait = nullptr;
                    inTransition = false;
                    transitionProgress = 0.0;
                }
                // If the transition has not completed then calculate the linearly interpolated gait frequency
                else {
                    gaitFrequency = primaryGaitFrequency + (transitionGaitFrequency - primaryGaitFrequency) * transitionProgress;
                }
            }
            else {
                gaitFrequency = primaryGaitFrequency;
            }

            // Update the phase clock and take the modulus so that it stays below 360 degrees
            phaseClock += std::fmod(360.0 * gaitFrequency / 50.0, 360.0);

            std::array<std::array<double, 3>, 4> footPositions;

            // For each of the legs, get the foot position
            for(uint16_t legNum = 0; legNum < 4; legNum++){
                footPositions[legNum] = get_foot_position(legNum);
            }

            return footPositions;
        }

        // Declare getters and setters
        // Function descriptions have not been provided since their purpose is self-descriptive
        gait* get_primary_gait() const { return primaryGait; }
        gait* get_transition_gait() const { return transitionGait; }

        void set_frequency_scale(double scale){
            frequencyScale = std::clamp(scale, 0.0, 1.0);
            return;
        }

        void set_turn_scale(double scale){
            controllerTurnScale = std::clamp(scale, -1.0, 1.0);
            return;
        }

        void set_roll_correction(double rollCorrection){
            this->rollCorrection = rollCorrection;
            return;
        }

        void set_pitch_correction(double pitchCorrection){
            this->pitchCorrection = pitchCorrection;
            return;
        }

        void set_enable_balance_offsets(bool enableBalanceOffsets) {
            this->enableBalanceOffsets = enableBalanceOffsets;
            return;
        }
        
        bool get_enable_balance_offsets() {
            return this->enableBalanceOffsets;
        }

        // End of getting and setter declarations

    private:

        // Define all of the gait control variables

        const double turnScaleIncrement = 1/100.0; // Controls how fast the turn scale will update

        double phaseClock = 0.0; // Global phase clocked used to determine the progress of the gait
        double gaitFrequency = 0.0; // How many times the gait should loop per second (Defined by gait parameters and controller)
        double transitionProgress = 0.0; // 0.0 - 1.0 Allows for linearly interpolating between gaits

        double frequencyScale = 1.0; // 0.0 - 1.0 Provided by the controller for controlling gait frequency
        double controllerTurnScale = 0.0; // -1.0 - 1.0 Provided by the controller for how agressive the robot should turn 
        double gaitTurnScale = 0.0; // Stores the active turn scale to prevent large changes from occuring at once

        // IMU PD controller balance updates
        double rollCorrection; 
        double pitchCorrection; 

        bool inTransition = false;

        bool enableBalanceOffsets = false;

        gait* primaryGait = nullptr;
        gait* transitionGait = nullptr;

        std::array<double, 3> get_curve_points(std::vector<std::vector<double>> bezierControlPoints, std::vector<double> curveOffsets, double t, double turnScale, std::vector<double> centerPoint) {
            
            /* ------
                Purpose:
                    Takes the bezier control points, curve offsets, time, turn scale, and center points and calulates the current foot position
                Parameters:
                    std::vector<std::vector<double>> bezierControlPoints - The points used to describe the bezier curve 
                    std::vector<double> curveOffsets - Provides the offset of where the foot should be from the bezier curve. This was used since the bezier curves were 
                                                       Defined at the origin, however, the control points themselves should really be updated
                    double t - The curve is parameterized from t=0.0 to t=1.0. This tells the function where to find the points
                    double turnScale - This will reduce the width of the bezier curve to allow the robot to turn
                    std::vector<double> centerPoint - This provides an axis along which the gait can be compressed
                Return 
                    std::array<double, 3> - The x, y, z coordinates of the end effector
               ------ */ 

            // ToDo: This function should also be updated so that it just handles bezier point calculation and not turning / offsets
            // ToDo: Hard code for n=5 and n=1 for now, however, actually implementing the general case should not be too difficult and would allow for more complex gaits in the future. Look into this later
            std::vector<std::function<double(double)>> bernsteinPolynomials5 = {
                [](double t) {return std::pow((1.0-t), 5);},
                [](double t) {return 5 * std::pow((1.0-t), 4) * t;},
                [](double t) {return 10 * std::pow((1.0-t), 3) * std::pow(t, 2);},
                [](double t) {return 10 * std::pow((1.0-t), 2) * std::pow(t, 3);},
                [](double t) {return 5 * std::pow((1.0-t), 1) * std::pow(t, 4);},
                [](double t) {return std::pow((t), 5);}
            };

            std::vector<std::function<double(double)>> bernsteinPolynomials1 = {
                [](double t) {return 1.0-t;},
                [](double t) {return t;}
            };

            std::array<double, 3> position = {0.0, 0.0, 0.0};

            std::vector<std::function<double(double)>> bernsteinPolynomials;

            // Based on the number of control points choose the vector of functions which matches
            if(bezierControlPoints.size() == 6){
                bernsteinPolynomials = bernsteinPolynomials5;
            } 
            else if(bezierControlPoints.size() == 2){
                bernsteinPolynomials = bernsteinPolynomials1;
            }

            // Calculate the positions of the end effector
            for(uint16_t index = 0; index < bezierControlPoints.size(); index++)
            {
                double polynomialFactor = bernsteinPolynomials[index](t);
                position[0] += polynomialFactor * bezierControlPoints[index][0];
                position[1] += polynomialFactor * bezierControlPoints[index][1];
                position[2] += polynomialFactor * bezierControlPoints[index][2];
            }  

            // Apply the curve offsets and turning scale
            position[0] = (position[0] - ((position[0] - centerPoint[0]) * 0.5 * turnScale)) + curveOffsets[0];
            position[1] = position[1] + curveOffsets[1];
            position[2] = position[2] + curveOffsets[2];

            return position;
        }

        void check_points(std::array<double, 3> footPosition){

            /* ------
                Purpose:
                    Checks to see if the end effector is in a valid position that it can reach
                Parameters:
                    std::array<double, 3> footPosition - The target coordiantes
                Return 
                    None
               ------ */ 

            // This function is currently not in use.

            // ToDo: Currently this just handles points along the xy plane of the leg. Extend an extra dimension
            if(10000 < std::pow((footPosition[0] + 56), 2) + std::pow((footPosition[1] + 102), 2)) throw std::runtime_error("Invalid Position");
            if(8900 > std::pow((footPosition[0] + 120), 2) + std::pow((footPosition[1] + 21), 2)) throw std::runtime_error("Invalid Position"); 
            if(13800 > std::pow((footPosition[0] - 58), 2) + std::pow((footPosition[1] + 12), 2)) throw std::runtime_error("Invalid Position"); 
        }
};

class control : public rclcpp::Node {  
public:
    control() : Node("control_node") {
        RCLCPP_INFO(this->get_logger(), "Control Node Started");

        // Get the gait parameters stored in the .YAML file
        idle = get_gait_params(*this, "idle");
        forwardTrot = get_gait_params(*this, "forwardTrot");
        forwardWalk = get_gait_params(*this, "forwardWalk");
        reverseWalk = get_gait_params(*this, "reverseWalk");
        rest = get_gait_params(*this, "rest");

        // Queue the rest gait by default
        gaitControl.queue_gait(&rest);

        // FL1 - Front, Left, 1st Motor
        // 1st Motor - Shoulder Joint
        // 2nd Motor - Upper Joint
        // 3rd Motor - Lower Elbow Joint
        servoMap = {
            {"servoFL1", 0}, {"servoFL2", 1}, {"servoFL3", 2},
            {"servoFR1", 4}, {"servoFR2", 5}, {"servoFR3", 6},
            {"servoBR1", 8}, {"servoBR2", 9}, {"servoBR3", 10},
            {"servoBL1", 12}, {"servoBL2", 13}, {"servoBL3", 14}
        };

        // This is used in the event that all of the servos need to be updated and provides and easy way to iterate over them
        // The update servo function should really be modified to take a vector of integers instead of strings so that this is not needed
        servoNames = {
            "servoFL1", "servoFL2", "servoFL3", 
            "servoFR1", "servoFR2", "servoFR3", 
            "servoBR1", "servoBR2", "servoBR3", 
            "servoBL1", "servoBL2", "servoBL3"
        };

        // Define all of the ROS2 subscriptions, publishers, and timers

        // Recieves messages from the connected controller
        _subController = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Joy::SharedPtr msg){
                input_handler(*msg);
            }
        );

        // Used for manual testing. Recieves a list of motors and angles and updates them accordingly
        _subMove = this->create_subscription<interfaces::msg::MotorTesting>(
            "control/moveServos", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting::SharedPtr msg){
                move_servos(*msg);
            }
        );  

        // Recieves balance correction updates from the balance node and uses them when calculating end effector location
        _subBalanceCorrection = this->create_subscription<interfaces::msg::BalanceCorrection>(
            "control/balanceCorrection", rclcpp::SensorDataQoS(),
            [this](interfaces::msg::BalanceCorrection::SharedPtr msg){
                update_balance_correction(*msg);
            }
        ); 

        // This publisher sends motor position messages to the PCA9685 Node to update the servo positions 
        _publisher = this->create_publisher<interfaces::msg::MotorPosition>(
            "servo/command", 10
        );

        // This publisher is used to send position information to the Gazebo sim
        _jointPublisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10
        );

        // This publisher sends a message to balance node to reset the offsets generated by PD. This stops the servos
        // From jumpting to a new position when it is enabled 
        _resetBalancePublisher = this->create_publisher<interfaces::msg::MotorPosition>(
            "/balance/reset_balance", 10
        );

        // This timer runs next tick at 50Hz to update the leg positions
        _timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 50.0),
            [this]() { next_tick(); }
        );

        // Get the motor parameters from the .yaml file
        declare_motor_parameters(*this);
        motorParameters = get_motor_parameters(*this);
    }

private:
    // Define all of the subscriptions, publishers, and timers. Their uses are defined above
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subController;
    rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subMove;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subReset;
    rclcpp::Subscription<interfaces::msg::BalanceCorrection>::SharedPtr _subBalanceCorrection;

    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _publisher;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _jointPublisher;
    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _resetBalancePublisher;

    rclcpp::TimerBase::SharedPtr _timer;

    // Create variables to store servo name and id
    std::unordered_map<std::string, uint8_t> servoMap;
    std::array<std::string, 12> servoNames;

    std::array<MotorData, 16> motorParameters;

    // Create a globalGaitControl object which can be called to get updated leg positions
    globalGaitControl gaitControl{};

    // Define gaits that can be used by the robot
    gait idle;
    gait forwardTrot;
    gait forwardWalk;
    gait reverseWalk;
    gait rest;

    // This array is used for checking if buttons have been released so they can be treated as toggle buttons
    std::array<bool, 12> buttonReset = {
        false, false, false, false,
        false, false, false, false,
        false, false, false, false
    };

    // Declare constant linkage lengths and angles (rad). See MZ for joint defenitions 
    const std::array<double, 8> linkLen = {24.0, 20.0, 38.0, 25.0, 95.0, 122.7};
    const std::array<double, 2> linkAng = {2.35619};

    const std::array<double, 2> motorPos = {-20, -20};

    double inv_cos_law(double sideA, double sideB, double sideC) const {

        /* ------
            Purpose:
                Calculates the angle C in radians. sideA and sideB are interchangeable 
            Parameters:
                double sideA - Length of side A 
                double sideB - Length of side B
                double sideC - Length of side C
            Return 
                double - Angle C in radians
            ------ */ 

        return std::acos((std::pow(sideC, 2) - std::pow(sideA, 2) - std::pow(sideB, 2))/(-2 * sideA * sideB));
    }

    std::array<double, 4> calc_joint_angles(std::array<double, 3> footPosition) const{
        /* ------
            Purpose:
                Inverse kinematics to calculate the joint angles requried to reach a certian position
            Parameters:
                std::array<double, 3> footPosition - The provided foot position that the motors should move towards
            Return 
                std::array<double, 4> - The angle of each motor on the robot leg + the angle of the bottom leg linkage which is used for Gazebo
            ------ */ 

        // Declare arrays to hold calculated intermediate values 
        std::array<double, 2> calcLen = {};
        std::array<double, 7> calcAng = {};

        calcLen[0] = std::sqrt(std::pow(footPosition[0], 2) + std::pow(footPosition[1], 2));
        calcAng[0] = std::atan(std::abs(footPosition[1]) / std::abs(footPosition[0]));

        if(footPosition[0] > 0) calcAng[0] = M_PI - calcAng[0];

        calcAng[1] = inv_cos_law(calcLen[0], linkLen[5], linkLen[4]);
        calcAng[2] = calcAng[0] - calcAng[1];
        calcAng[3] = M_PI - linkAng[0] - calcAng[2];

        double x2 = -linkLen[2] * std::cos(calcAng[3]) - motorPos[0];
        double y2 = linkLen[2] * std::sin(calcAng[3]) - motorPos[1];

        calcLen[1] = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
        calcAng[4] = std::atan(std::abs(y2)/std::abs(x2));
        calcAng[5] = inv_cos_law(calcLen[1], linkLen[0], linkLen[1]);
        calcAng[6] = calcAng[4] - calcAng[5];

        double motor2Ang = std::atan(
            std::abs(footPosition[1]+linkLen[5]*std::sin(calcAng[2]))/
            std::abs(footPosition[0]+linkLen[5]*std::cos(calcAng[2])));

        double motorCalfAng = motor2Ang + calcAng[2] - M_PI_2;

        // All returned values should be stored in calcAng and not in seperate variables. Also update drawing to rep changes.
        return std::array<double, 4> {0, motor2Ang, calcAng[6], motorCalfAng};
    }

    void next_tick() {

        /* ------
            Purpose:
                Gets the foot positions and calls to update the motor posititons
            Parameters:
                None
            Return 
                None
            ------ */ 

        // Get foot positions
        std::array<std::array<double, 3>, 4> footPosition = gaitControl.run_next_tick();

        std::vector<double> vJointAngles = {};

        // Get the joint angles based on the foot position for each leg
        for (uint16_t legNum = 0; legNum < 4; legNum++){
            std::array<double, 4> jointAngles = calc_joint_angles(footPosition[legNum]);
            vJointAngles.insert(vJointAngles.end(), jointAngles.begin(), jointAngles.end());
        }

        update_servo_positions(std::vector<std::string>{
            "servoFL1", "servoFL2", "servoFL3", "servoFLCalf", 
            "servoFR1", "servoFR2", "servoFR3", "servoFRCalf",
            "servoBL1", "servoBL2", "servoBL3", "servoBLCalf",
            "servoBR1", "servoBR2", "servoBR3", "servoBRCalf"}, 
            vJointAngles);

        return;
    }

    void update_servo_positions(std::vector<std::string> names, std::vector<double> angles){

        /* ------
            Purpose:
                Takes the names of each motor and the target angles and sends a message to the PCA9685 node to update them 
            Parameters:
                std::vector<std::string> names - The names of each motor that should be updated (e.g servoFL1)
                std::vector<double> angles - The angles corresponding to each of the names provided
            Return
                None
            ------ */ 

        interfaces::msg::MotorPosition message;
        
        std::vector<uint16_t> motors;
        std::vector<double> pulses;
        
        // Create a joint trajectory and joint trajectory point for Gazebo
        trajectory_msgs::msg::JointTrajectory jointMessage;
        trajectory_msgs::msg::JointTrajectoryPoint trajectoryPoint;

        // Set that it should take 1/50s to update to the new position
        trajectoryPoint.time_from_start.sec = 1/50.0;

        // Loop over each servo that was provided
        for(uint16_t i = 0; i < names.size(); i++){

            // Format data for Gazebo joints. We are not simulating the entire linkage system so a calculated calf angle 
            // needs to be provided
            if(names[i].back() != '3'){
                jointMessage.joint_names.push_back(names[i]);
                // If updating the second motor then the angle should be inverted
                if(names[i].back() == '2')
                {
                    trajectoryPoint.positions.push_back(angles[i] * -1);
                }
                else {
                    trajectoryPoint.positions.push_back(angles[i]);
                }
                
            }

            double angle = angles[i];

            // Find the servo data based on the servo name
            auto servo = servoMap.find(names[i]);

            // If the value doesn't exist in the servo map then skip the motor (Mostly for sim calf joint)
            if(servo == servoMap.end()){
                continue;
            }

            const auto& servoNum = servo->second;
            MotorData servoParams = motorParameters[servoNum];

            // Update the servo angle based on the parameters
            if(servoParams.inverted) angle = angle * -1.0;  // Servos on the opposite side of the robot move in the opposite direction
            
            angle += servoParams.offset; // Add the calibration offset
            angle = std::clamp(angle, servoParams.minAngle, servoParams.maxAngle); // Clamp the angle based on the max and min parameters
    
            motors.push_back(servoNum);
            pulses.push_back(rad_to_us(angle, servoParams.minUs, servoParams.maxUs));
        }

        // Add the joint trajectory point to the joint message
        jointMessage.points.push_back(trajectoryPoint);

        // Publish the value to Gazebo
        _jointPublisher->publish(jointMessage);

        message.motor = motors;
        message.pulses = pulses;

        // Publish the value to the PCA9685 Node
        _publisher->publish(message);
    }

    void move_servos(const interfaces::msg::MotorTesting &msg){
        /* ------
            Purpose:
                This function is used for testing. It will take in a vector or motor names and angles and force the update
            Parameters:
                const interfaces::msg::MotorTesting &msg - Contains a vector for motor and angle data
            Return
                None
            ------ */ 

        update_servo_positions(msg.motors, msg.angles);   
    }

    void update_balance_correction(const interfaces::msg::BalanceCorrection &msg){
        /* ------
            Purpose:
                This function takes in a BalanceCorrection message and updates the pitch and roll correction values for stabilization
            Parameters:
                const interfaces::msg::BalanceCorrection &msg - Message containing pitch and roll correction values
            Return
                None
            ------ */ 

        gaitControl.set_roll_correction(msg.roll_correction);
        gaitControl.set_pitch_correction(msg.pitch_correction);
    }

    void input_handler(const sensor_msgs::msg::Joy &msg){

        /* ------
            Purpose:
                This handles all of the input logic from a controller and updates the global gate handler accordingly 
            Parameters:
                sensor_msgs::msg::Joy &msg - This message contains all of the controller data that ROS2 provides
            Return
                None
            ------ */ 

        // Set the frequency and turn scale based on the appropriate joystick axis
        gaitControl.set_frequency_scale(std::abs(msg.axes[1])); // Vertical left stick
        gaitControl.set_turn_scale(msg.axes[3]); // Horizontal right stick

        // get the primary and transition gaits
        gait* activePrimaryGait = gaitControl.get_primary_gait();
        gait* activeTransitionGait = gaitControl.get_transition_gait(); 

        // Handle Transitions
        // ToDo: Each gait should ideally have a list of gaits that it can transition from so that the second nested
        //       if statement for each branch doesn't have to check
        if(msg.axes[1] > 0.2 && msg.axes[1] < 0.5) {
            if(activePrimaryGait == &idle && activeTransitionGait != &forwardWalk)
            {
                gaitControl.queue_gait(&forwardWalk);
            }
        } 
        else if(msg.axes[1] > 0.5) {
            if(activePrimaryGait == &forwardWalk && activeTransitionGait != &forwardTrot)
            {
                gaitControl.queue_gait(&forwardTrot);
            }
        }
        else if(msg.axes[1] < -0.2){
            if(activePrimaryGait == &idle && activeTransitionGait != &reverseWalk)
            {
                gaitControl.queue_gait(&reverseWalk);
            }
        }
        else if(std::abs(msg.axes[1]) <= 0.2)
        {
            if(activePrimaryGait != &idle && activePrimaryGait != &rest && activeTransitionGait != &idle)
            {
                gaitControl.queue_gait(&idle);
            }    
        }

        // Handle button inputs
        // If the A button is pressed then balance offsets should be reset and toggled on or off
        if(msg.buttons[0] == 1 && buttonReset[0])
        {
            buttonReset[0] = false;

            interfaces::msg::MotorPosition msg;
            _resetBalancePublisher->publish(msg);

            gaitControl.set_enable_balance_offsets(!gaitControl.get_enable_balance_offsets());
        }
        else if(msg.buttons[0] == 0 && !buttonReset[0]){
            buttonReset[0] = true;
        }

        // If the B button is pressed then the rest gate should be queued if possible
        if(msg.buttons[1] == 1 && buttonReset[1]){
            if(activePrimaryGait == &rest) gaitControl.queue_gait(&idle);
            else if(activePrimaryGait == &idle) gaitControl.queue_gait(&rest);
            buttonReset[1] = false;
        }
        else if(msg.buttons[1] == 0 && !buttonReset[1]){
            buttonReset[1] = true;
        }
        
    }

    double rad_to_us(double rad, double minUs, double maxUs) const {
        
        /* ------
            Purpose:
                This function takes the desired angle in radians and converts it to a PWM pulse in microseconds
            Parameters:
                double rad - The target angle
                double minUs - The minimum pulse size in microseconds
                double maxUs - The maximum pulse size in microseconds
            Return
                double - The pulse duration in microseconds
            ------ */ 

        // Normalize the angle
        // Note: Currently the motors have ~20 degrees taken off of their max range. This was just to protect
        //       the motors when testing. 
        // ToDo: Make sure the motors won't stall when going to +- 90 degrees when the 20 degree reduction is removed
        double t = (rad + (M_PI_2 - 0.174533)) / (M_PI - 0.349066);

        // If the normalized value exceeds its range then limit it
        if(t < 0.0) t = 0.0;
        if(t > 1.0) t = 1.0;

        // Map the normalized angle to a pulse duration
        return minUs + t * (maxUs - minUs);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();

    return 0;
}
    