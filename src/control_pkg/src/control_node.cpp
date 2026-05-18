#include <cmath>
#include <algorithm>

#include "control_pkg/control_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/int.hpp"
#include "interfaces/msg/double.hpp"
#include "interfaces/msg/motor_position.hpp"
#include "interfaces/msg/motor_testing.hpp"
#include "interfaces/msg/balance_correction.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "sensor_msgs/msg/joy.hpp"

class globalGaitControl {
    public:
        bool enableBalanceOffsets = false;

        globalGaitControl() = default;

        void queue_gait(gait *gait){
            if(primaryGait == nullptr) {
                primaryGait = gait;
            }
            else if(transitionGait == nullptr){
                transitionGait = gait;
                inTransition = true;
            }
        }

        std::array<double, 3> get_foot_position(uint16_t legNum){

            if(primaryGait == nullptr) {
                return std::array<double, 3>{-20.0, -155.0, 0};
            }

            double legTurnScale = 0.0;
            if(gaitTurnScale < 0.0 && (legNum == 1 || legNum == 3)) legTurnScale = std::abs(gaitTurnScale);
            else if(gaitTurnScale > 0.0 && (legNum == 0 || legNum == 2)) legTurnScale = std::abs(gaitTurnScale);

            std::vector<int64_t> phaseOffsets = primaryGait->get_phase_offset();

            double primaryPhase = std::fmod(phaseClock + phaseOffsets[legNum], 360.0);
            double primarySwingSwitchPhase = primaryGait->get_swing_switch_phase();

            std::array<double, 3> primaryPosition;

            if(primaryPhase <= primarySwingSwitchPhase){
                primaryPosition = get_curve_points(
                    primaryGait->get_swing_control_points(), 
                    primaryGait->get_curve_offsets(), 
                    primaryPhase / primarySwingSwitchPhase,
                    legTurnScale,
                    primaryGait->get_center_point()
                );
            } 
            else{
                primaryPosition = get_curve_points(
                    primaryGait->get_stance_control_points(), 
                    primaryGait->get_curve_offsets(), 
                    (primaryPhase - primarySwingSwitchPhase) / (360.0 - primarySwingSwitchPhase),
                    legTurnScale,
                    primaryGait->get_center_point()
                );
            } 

            if(inTransition){
                std::vector<int64_t> transitionPhaseOffsets = transitionGait->get_phase_offset();

                double transitionPhase = std::fmod(phaseClock + transitionPhaseOffsets[legNum], 360.0);
                double transitionSwingSwitchPhase = transitionGait->get_swing_switch_phase();

                std::array<double, 3> transitionPosition;
                
                if(transitionPhase <= transitionSwingSwitchPhase) transitionPosition = get_curve_points(
                    transitionGait->get_swing_control_points(), 
                    transitionGait->get_curve_offsets(), 
                    transitionPhase / transitionSwingSwitchPhase,
                    legTurnScale,
                    transitionGait->get_center_point()
                );
                else transitionPosition = get_curve_points(
                    transitionGait->get_stance_control_points(), 
                    transitionGait->get_curve_offsets(), 
                    (transitionPhase - transitionSwingSwitchPhase) / (360.0 - transitionSwingSwitchPhase),
                    legTurnScale,
                    transitionGait->get_center_point()
                );

                std::array<double, 3> blendedPosition = {
                    (1.0 - transitionProgress) * primaryPosition[0] + transitionProgress * transitionPosition[0],
                    (1.0 - transitionProgress) * primaryPosition[1] + transitionProgress * transitionPosition[1],
                    (1.0 - transitionProgress) * primaryPosition[2] + transitionProgress * transitionPosition[2]
                };
                
                if(enableBalanceOffsets)
                {
                    if(legNum == 0 || legNum == 1) blendedPosition[1] += pitchCorrection;
                    else blendedPosition[1] -= pitchCorrection;

                    if(legNum == 0 || legNum == 2) blendedPosition[1] -= rollCorrection;
                    else blendedPosition[1] += rollCorrection;
                }

                return blendedPosition;
            }
            else {
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

            if(controllerTurnScale < gaitTurnScale) gaitTurnScale = std::max(gaitTurnScale - turnScaleIncrement, controllerTurnScale);
            else if(controllerTurnScale > gaitTurnScale) gaitTurnScale = std::min(gaitTurnScale + turnScaleIncrement, controllerTurnScale);

            double primaryGaitFrequency = 0.1;

            if(primaryGait != nullptr) {
                primaryGaitFrequency = primaryGait->get_min_gait_frequency() + (primaryGait->get_max_gait_frequency() - primaryGait->get_min_gait_frequency()) * frequencyScale;
            }

            if(inTransition){
                double transitionGaitFrequency = transitionGait->get_min_gait_frequency() + (transitionGait->get_max_gait_frequency() - transitionGait->get_min_gait_frequency()) * frequencyScale;

                transitionProgress += 1 / 50.0; // This should be based on the time since the last tick and the desired transition time
                transitionProgress = std::min(transitionProgress, 1.0);

                if(transitionProgress >= 1.0){
                    primaryGait = transitionGait;
                    transitionGait = nullptr;
                    inTransition = false;
                    transitionProgress = 0.0;
                }
                else {
                    gaitFrequency = primaryGaitFrequency + (transitionGaitFrequency - primaryGaitFrequency) * transitionProgress;
                }
            }
            else {
                gaitFrequency = primaryGaitFrequency;
            }

            phaseClock += std::fmod(360.0 * gaitFrequency / 50.0, 360.0); // This might cause a weird slowing effect. Look into this later

            std::array<std::array<double, 3>, 4> footPositions;

            for(uint16_t legNum = 0; legNum < 4; legNum++){
                footPositions[legNum] = get_foot_position(legNum);
            }

            return footPositions;
        }

        gait* get_primary_gait() const { return primaryGait; }
        gait* get_transition_gait() const { return transitionGait; }

        void set_frequency_scale(double scale){
            frequencyScale = std::clamp(scale, 0.0, 1.0);
        }

        void set_turn_scale(double scale){
            controllerTurnScale = std::clamp(scale, -1.0, 1.0);
        }

        void set_roll_correction(double rollCorrection){
            this->rollCorrection = rollCorrection;
        }

        void set_pitch_correction(double pitchCorrection){
            this->pitchCorrection = pitchCorrection;
        }
        
    private:
        const double turnScaleIncrement = 1/100.0;

        double phaseClock = 0.0;
        double gaitFrequency = 0.0;
        double transitionProgress = 0.0;

        double frequencyScale = 1.0;
        double controllerTurnScale = 0.0;
        double gaitTurnScale = 0.0;

        double stanceSwitchPrimary = 180.0;
        double stanceSwitchTransition = 180.0;

        double rollCorrection;
        double pitchCorrection;

        bool inTransition = false;

        gait* primaryGait = nullptr;
        gait* transitionGait = nullptr;

        std::array<double, 3> get_curve_points(std::vector<std::vector<double>> bezierControlPoints, std::vector<double> curveOffsets, double t, double turnScale, std::vector<double> centerPoint)
        {
            // This function should also be updated so that it just handles bezier point calculation and not turning / offsets
            // Hard code for n=5 and n=1 for now, however, actually implementing the general case should not be too difficult and would allow for more complex gaits in the future. Look into this later
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

            if(bezierControlPoints.size() == 6){
                bernsteinPolynomials = bernsteinPolynomials5;
            } 
            else if(bezierControlPoints.size() == 2){
                bernsteinPolynomials = bernsteinPolynomials1;
            }

            for(uint16_t index = 0; index < bezierControlPoints.size(); index++)
            {
                double polynomialFactor = bernsteinPolynomials[index](t);
                position[0] += polynomialFactor * bezierControlPoints[index][0];
                position[1] += polynomialFactor * bezierControlPoints[index][1];
                position[2] += polynomialFactor * bezierControlPoints[index][2];
            }  

            position[0] = (position[0] - ((position[0] - centerPoint[0]) * 0.5 * turnScale)) + curveOffsets[0];
            position[1] = position[1] + curveOffsets[1];
            position[2] = position[2] + curveOffsets[2];

            return position;
        }

        void check_points(std::array<double, 3> footPosition){
            // Currently this just handles points along the xy plane of the leg
            // Extend an extra dimension
            
            if(10000 < std::pow((footPosition[0] + 56), 2) + std::pow((footPosition[1] + 102), 2)) throw std::runtime_error("Invalid Position");
            if(8900 > std::pow((footPosition[0] + 120), 2) + std::pow((footPosition[1] + 21), 2)) throw std::runtime_error("Invalid Position"); 
            if(13800 > std::pow((footPosition[0] - 58), 2) + std::pow((footPosition[1] + 12), 2)) throw std::runtime_error("Invalid Position"); 
        }
};

class control : public rclcpp::Node {  
public:
    control() : Node("control_node") {
        RCLCPP_INFO(this->get_logger(), "Control Node Started");

        // Get the gait parameters stroed in the .YAML file
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

        _subController = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Joy::SharedPtr msg){
                input_handler(*msg);
            }
        );

        _subMove = this->create_subscription<interfaces::msg::MotorTesting>(
            "control/moveServos", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting::SharedPtr msg){
                move_servos(*msg);
            }
        );  

        _subBalanceCorrection = this->create_subscription<interfaces::msg::BalanceCorrection>(
            "control/balanceCorrection", rclcpp::SensorDataQoS(),
            [this](interfaces::msg::BalanceCorrection::SharedPtr msg){
                update_balance_correction(*msg);
            }
        ); 

        _publisher = this->create_publisher<interfaces::msg::MotorPosition>(
            "servo/command", 10
        );

        _jointPublisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10
        );

        _resetBalancePublisher = this->create_publisher<interfaces::msg::MotorPosition>(
            "/balance/reset_balance", 10
        );

        _timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 50.0),
            [this]() { next_tick(); }
        );

        declare_motor_parameters(*this);
        motorParameters = get_motor_parameters(*this);
    }

private:
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subController;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subTrot;
    rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subMove;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subReset;
    rclcpp::Subscription<interfaces::msg::BalanceCorrection>::SharedPtr _subBalanceCorrection;

    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _publisher;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _jointPublisher;
    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _resetBalancePublisher;

    rclcpp::TimerBase::SharedPtr _timer;

    std::unordered_map<std::string, uint8_t> servoMap;
    std::array<std::string, 12> servoNames;

    std::array<MotorData, 16> motorParameters;

    globalGaitControl gaitControl{};

    gait idle;
    gait forwardTrot;
    gait forwardWalk;
    gait reverseWalk;
    gait rest;

    std::array<bool, 12> buttonReset = {
        false, false, false, false,
        false, false, false, false,
        false, false, false, false
    };

    // Declare constant linkage lengths and angles (rad). See MZ for joint defenitions 
    const std::array<double, 8> linkLen = {24.0, 20.0, 38.0, 25.0, 95.0, 122.7};
    const std::array<double, 2> linkAng = {2.35619};

    const std::array<double, 2> motorPos = {-20, -20};

    double inv_cos_law(double sideA, double sideB, double sideC) {
        // This function returns angle C in radians
        // sideA and sideB are interchangable 
        return std::acos((std::pow(sideC, 2) - std::pow(sideA, 2) - std::pow(sideB, 2))/(-2 * sideA * sideB));
    }

    std::array<double, 4> calc_joint_angles(std::array<double, 3> footPosition){
        // Inverse kinematics to calculate the joint angles requried to reach a certian position

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
        std::array<std::array<double, 3>, 4> footPosition = gaitControl.run_next_tick();

        std::vector<double> vJointAngles = {};

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

        interfaces::msg::MotorPosition message;
        
        std::vector<uint16_t> motors;
        std::vector<double> pulses;
        
        trajectory_msgs::msg::JointTrajectory jointMessage;
        trajectory_msgs::msg::JointTrajectoryPoint trajectoryPoint;

        trajectoryPoint.time_from_start.sec = 1/50.0;

        // Loop over each servo that was provided
        for(uint16_t i = 0; i < names.size(); i++){

            // Format data for Gazebo joints. We are not simulating the entire linkage system so a calculated calf angle 
            // needs to be provided
            if(names[i].back() != '3'){
                jointMessage.joint_names.push_back(names[i]);
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
            
            angle += servoParams.offset;           // Add the calibration offset
            angle = std::clamp(angle, servoParams.minAngle, servoParams.maxAngle); // Clamp the angle based on the max and min parameters
    
            motors.push_back(servoNum);
            pulses.push_back(rad_to_us(angle, servoParams.minUs, servoParams.maxUs));
        }

        jointMessage.points.push_back(trajectoryPoint);

        _jointPublisher->publish(jointMessage);

        message.motor = motors;
        message.pulses = pulses;

        //RCLCPP_INFO(this->get_logger(), "Sending Message");

        _publisher->publish(message);
    }

    void move_servos(const interfaces::msg::MotorTesting &msg){
        update_servo_positions(msg.motors, msg.angles);   
    }

    void update_balance_correction(const interfaces::msg::BalanceCorrection &msg){
        gaitControl.set_roll_correction(msg.roll_correction);
        gaitControl.set_pitch_correction(msg.pitch_correction);
    }

    void input_handler(const sensor_msgs::msg::Joy &msg){

        // Set the frequency and turn scale based on the appropriate joystick axis
        gaitControl.set_frequency_scale(std::abs(msg.axes[1]));
        gaitControl.set_turn_scale(msg.axes[3]);

        // get the primary and transition gaits
        gait* activePrimaryGait = gaitControl.get_primary_gait();
        gait* activeTransitionGait = gaitControl.get_transition_gait(); 

        // Handle Transitions
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

        if(msg.buttons[0] == 1 && buttonReset[0])
        {
            buttonReset[0] = false;

            interfaces::msg::MotorPosition msg;
            _resetBalancePublisher->publish(msg);

            gaitControl.enableBalanceOffsets = !gaitControl.enableBalanceOffsets;
        }
        else if(msg.buttons[0] == 0 && !buttonReset[0]){
            buttonReset[0] = true;
        }

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
        
        // Convert radians to pulse length in microseconds
        double t = (rad + (M_PI_2 - 0.174533)) / (M_PI - 0.349066);
        if(t < 0.0) t = 0.0;
        if(t > 1.0) t = 1.0;
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
    