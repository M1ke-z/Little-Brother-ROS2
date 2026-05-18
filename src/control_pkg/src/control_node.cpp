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

class global_gait_control {
    public:
        bool enableBalanceOffsets = false;

        global_gait_control() = default;

        void queue_gait(gait *gait){
            if(primaryGait == nullptr) {
                primaryGait = gait;
            }
            else if(transitionGait == nullptr){
                transitionGait = gait;
                inTransition = true;
            }
        }

        std::array<double, 3> get_foot_position(uint16_t leg_num){

            if(primaryGait == nullptr) {
                return std::array<double, 3>{-20.0, -155.0, 0};
            }

            double legTurnScale = 0.0;
            if(gaitTurnScale < 0.0 && (leg_num == 1 || leg_num == 3)) legTurnScale = std::abs(gaitTurnScale);
            else if(gaitTurnScale > 0.0 && (leg_num == 0 || leg_num == 2)) legTurnScale = std::abs(gaitTurnScale);

            std::vector<int64_t> phaseOffsets = primaryGait->getPhaseOffset();

            double primaryPhase = std::fmod(phaseClock + phaseOffsets[leg_num], 360.0);

            std::array<double, 3> primary_position;

            if(primaryPhase <= primaryGait->getSwingSwitchPhase()){
                primary_position = get_curve_points(
                    primaryGait->getSwingControlPoints(), 
                    primaryGait->getCurveOffsets(), 
                    primaryPhase / primaryGait->getSwingSwitchPhase(),
                    legTurnScale,
                    primaryGait->getCenterPoint()
                );
            } 
            else{
                primary_position = get_curve_points(
                    primaryGait->getStanceControlPoints(), 
                    primaryGait->getCurveOffsets(), 
                    (primaryPhase - primaryGait->getSwingSwitchPhase()) / (360.0 - primaryGait->getSwingSwitchPhase()),
                    legTurnScale,
                    primaryGait->getCenterPoint()
                );
            } 

            if(inTransition){
                std::vector<int64_t> transitionPhaseOffsets = transitionGait->getPhaseOffset();
                double transitionPhase = std::fmod(phaseClock + transitionPhaseOffsets[leg_num], 360.0);

                std::array<double, 3> transition_position;
                
                if(transitionPhase <= transitionGait->getSwingSwitchPhase()) transition_position = get_curve_points(
                    transitionGait->getSwingControlPoints(), 
                    transitionGait->getCurveOffsets(), 
                    transitionPhase / transitionGait->getSwingSwitchPhase(),
                    legTurnScale,
                    transitionGait->getCenterPoint()
                );
                else transition_position = get_curve_points(
                    transitionGait->getStanceControlPoints(), 
                    transitionGait->getCurveOffsets(), 
                    (transitionPhase - transitionGait->getSwingSwitchPhase()) / (360.0 - transitionGait->getSwingSwitchPhase()),
                    legTurnScale,
                    transitionGait->getCenterPoint()
                );

                std::array<double, 3> blended_position = {
                    (1.0 - transitionProgress) * primary_position[0] + transitionProgress * transition_position[0],
                    (1.0 - transitionProgress) * primary_position[1] + transitionProgress * transition_position[1],
                    (1.0 - transitionProgress) * primary_position[2] + transitionProgress * transition_position[2]
                };
                
                if(enableBalanceOffsets)
                {
                    if(leg_num == 0 || leg_num == 1) blended_position[1] += pitch_correction;
                    else blended_position[1] -= pitch_correction;

                    if(leg_num == 0 || leg_num == 2) blended_position[1] -= roll_correction;
                    else blended_position[1] += roll_correction;
                }

                //check_points(blended_position);

                return blended_position;
            }
            else {
                if(enableBalanceOffsets)
                {
                    //std::cout << "Roll Correction: " << roll_correction << ", Pitch Correction: " << pitch_correction << std::endl;

                    if(leg_num == 0 || leg_num == 1) primary_position[1] += pitch_correction;
                    else primary_position[1] -= pitch_correction;

                    if(leg_num == 0 || leg_num == 2) primary_position[1] -= roll_correction;
                    else primary_position[1] += roll_correction;
                }

                //check_points(primary_position);

                return primary_position;
            }
        }

        std::array<std::array<double, 3>, 4> runNextTick(){

            if(controllerTurnScale < gaitTurnScale) gaitTurnScale = std::max(gaitTurnScale - turnScaleIncrement, controllerTurnScale);
            else if(controllerTurnScale > gaitTurnScale) gaitTurnScale = std::min(gaitTurnScale + turnScaleIncrement, controllerTurnScale);

            double primaryGaitFrequency = 0.1;

            if(primaryGait != nullptr) {
                primaryGaitFrequency = primaryGait->getMinGaitFrequency() + (primaryGait->getMaxGaitFrequency() - primaryGait->getMinGaitFrequency()) * frequencyScale;
            }

            if(inTransition){
                double transitionGaitFrequency = transitionGait->getMinGaitFrequency() + (transitionGait->getMaxGaitFrequency() - transitionGait->getMinGaitFrequency()) * frequencyScale;

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

            std::array<std::array<double, 3>, 4> foot_positions;

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++){
                foot_positions[leg_num] = get_foot_position(leg_num);
            }

            return foot_positions;
        }

        gait* get_primary_gait() const { return primaryGait; }
        gait* get_transition_gait() const { return transitionGait; }

        void set_frequency_scale(double scale){
            frequencyScale = std::clamp(scale, 0.0, 1.0);
        }

        void set_turn_scale(double scale){
            controllerTurnScale = std::clamp(scale, -1.0, 1.0);
        }

        void set_roll_correction(double roll_correction){
            this->roll_correction = roll_correction;
        }

        void set_pitch_correction(double pitch_correction){
            this->pitch_correction = pitch_correction;
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

        double roll_correction;
        double pitch_correction;

        bool inTransition = false;

        gait* primaryGait = nullptr;
        gait* transitionGait = nullptr;

        std::array<double, 3> get_curve_points(std::vector<std::vector<double>> bezierControlPoints, std::vector<double> curveOffsets, double t, double turnScale, std::vector<double> centerPoint)
        {
            // This function should also be updated so that it just handles bezier point calculation and not turning / offsets
            // Hard code for n=5 and n=1 for now, however, actually implementing the general case should not be too difficult and would allow for more complex gaits in the future. Look into this later
            std::vector<std::function<double(double)>> bernstein_polynomials_5 = {
                [](double t) {return std::pow((1.0-t), 5);},
                [](double t) {return 5 * std::pow((1.0-t), 4) * t;},
                [](double t) {return 10 * std::pow((1.0-t), 3) * std::pow(t, 2);},
                [](double t) {return 10 * std::pow((1.0-t), 2) * std::pow(t, 3);},
                [](double t) {return 5 * std::pow((1.0-t), 1) * std::pow(t, 4);},
                [](double t) {return std::pow((t), 5);}
            };

            std::vector<std::function<double(double)>> bernstein_polynomials_1 = {
                [](double t) {return 1.0-t;},
                [](double t) {return t;}
            };

            std::array<double, 3> position = {0.0, 0.0, 0.0};

            std::vector<std::function<double(double)>> bernstein_polynomials;

            if(bezierControlPoints.size() == 6){
                bernstein_polynomials = bernstein_polynomials_5;
            } 
            else if(bezierControlPoints.size() == 2){
                bernstein_polynomials = bernstein_polynomials_1;
            }

            for(uint16_t index = 0; index < bezierControlPoints.size(); index++)
            {
                double polynomial_factor = bernstein_polynomials[index](t);
                position[0] += polynomial_factor * bezierControlPoints[index][0];
                position[1] += polynomial_factor * bezierControlPoints[index][1];
                position[2] += polynomial_factor * bezierControlPoints[index][2];
            }  

            position[0] = (position[0] - ((position[0] - centerPoint[0]) * 0.5 * turnScale)) + curveOffsets[0];
            position[1] = position[1] + curveOffsets[1];
            position[2] = position[2] + curveOffsets[2];

            return position;
        }

        void check_points(std::array<double, 3> foot_position){
            // Currently this just handles points along the plane of the leg
            // Extend an extra dimension
            

            // 400
            if(10000 < std::pow((foot_position[0] + 56), 2) + std::pow((foot_position[1] + 102), 2)) throw std::runtime_error("Invalid Position");
            if(8900 > std::pow((foot_position[0] + 120), 2) + std::pow((foot_position[1] + 21), 2)) throw std::runtime_error("Invalid Position"); 
            if(13800 > std::pow((foot_position[0] - 58), 2) + std::pow((foot_position[1] + 12), 2)) throw std::runtime_error("Invalid Position"); 
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
        gait_control.queue_gait(&rest);

        // These channels should be passed in as parameters rather than hardcoded. Look into this
        // FL1 - Front, Left, 1st Motor
        // 1st Motor - Shoulder Joint
        // 2nd Motor - Upper Joint
        // 3rd Motor - Lower Elbow Joint
        servo_map_ = {
            {"servoFL1", 0}, {"servoFL2", 1}, {"servoFL3", 2},
            {"servoFR1", 4}, {"servoFR2", 5}, {"servoFR3", 6},
            {"servoBR1", 8}, {"servoBR2", 9}, {"servoBR3", 10},
            {"servoBL1", 12}, {"servoBL2", 13}, {"servoBL3", 14}
        };

        // This is used in the event that all of the servos need to be updated and provides and easy way to iterate over them
        // The update servo function should really be modified to take a vector of integers instead of strings so that this is not needed
        servo_names = {
            "servoFL1", "servoFL2", "servoFL3", 
            "servoFR1", "servoFR2", "servoFR3", 
            "servoBR1", "servoBR2", "servoBR3", 
            "servoBL1", "servoBL2", "servoBL3"
        };

        _subController = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Joy::SharedPtr msg){
                inputHandler(*msg);
            }
        );

        _subMove = this->create_subscription<interfaces::msg::MotorTesting>(
            "control/moveServos", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting::SharedPtr msg){
                moveServos(*msg);
            }
        );  

        _subBalanceCorrection = this->create_subscription<interfaces::msg::BalanceCorrection>(
            "control/balanceCorrection", rclcpp::SensorDataQoS(),
            [this](interfaces::msg::BalanceCorrection::SharedPtr msg){
                updateBalanceCorrection(*msg);
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

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 50.0),
            [this]() { nextTick(); }
        );

        declare_motor_parameters(*this);
        motor_parameters = get_motor_parameters(*this);
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

    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, uint8_t> servo_map_;
    std::array<std::string, 12> servo_names;

    std::array<MotorData, 16> motor_parameters;

    global_gait_control gait_control{};

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

    double invCosLaw(double sideA, double sideB, double sideC) {
        // This function returns angle C in radians
        // sideA and sideB are interchangable 
        return std::acos((std::pow(sideC, 2) - std::pow(sideA, 2) - std::pow(sideB, 2))/(-2 * sideA * sideB));
    }

    std::array<double, 4> calcJointAngles(std::array<double, 3> footPosition){
        // Inverse kinematics to calculate the joint angles requried to reach a certian position

        // Declare arrays to hold calculated intermediate values 
        std::array<double, 2> calcLen = {};
        std::array<double, 7> calcAng = {};

        calcLen[0] = std::sqrt(std::pow(footPosition[0], 2) + std::pow(footPosition[1], 2));
        calcAng[0] = std::atan(std::abs(footPosition[1]) / std::abs(footPosition[0]));

        if(footPosition[0] > 0) calcAng[0] = M_PI - calcAng[0];

        calcAng[1] = invCosLaw(calcLen[0], linkLen[5], linkLen[4]);
        calcAng[2] = calcAng[0] - calcAng[1];
        calcAng[3] = M_PI - linkAng[0] - calcAng[2];

        double x2 = -linkLen[2] * std::cos(calcAng[3]) - motorPos[0];
        double y2 = linkLen[2] * std::sin(calcAng[3]) - motorPos[1];

        calcLen[1] = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
        calcAng[4] = std::atan(std::abs(y2)/std::abs(x2));
        calcAng[5] = invCosLaw(calcLen[1], linkLen[0], linkLen[1]);
        calcAng[6] = calcAng[4] - calcAng[5];

        double motor2Ang = std::atan(
            std::abs(footPosition[1]+linkLen[5]*std::sin(calcAng[2]))/
            std::abs(footPosition[0]+linkLen[5]*std::cos(calcAng[2])));

        double motorCalfAng = motor2Ang + calcAng[2] - M_PI_2;

        // All returned values should be stored in calcAng and not in seperate variables. Also update drawing to rep changes.
        return std::array<double, 4> {0, motor2Ang, calcAng[6], motorCalfAng};
    }

    void nextTick() {
        std::array<std::array<double, 3>, 4> foot_position = gait_control.runNextTick();

        std::vector<double> v_joint_angles = {};

        for (uint16_t leg_num = 0; leg_num < 4; leg_num++){
            std::array<double, 4> joint_angles = calcJointAngles(foot_position[leg_num]);
            v_joint_angles.insert(v_joint_angles.end(), joint_angles.begin(), joint_angles.end());
        }

        // RCLCPP_INFO(this->get_logger(), "Servo1: %.2f, Servo2: %.2f, ServoCalf: %.2f", v_joint_angles[0], v_joint_angles[1], v_joint_angles[3]);

        update_servo_positions(std::vector<std::string>{
            "servoFL1", "servoFL2", "servoFL3", "servoFLCalf", 
            "servoFR1", "servoFR2", "servoFR3", "servoFRCalf",
            "servoBL1", "servoBL2", "servoBL3", "servoBLCalf",
            "servoBR1", "servoBR2", "servoBR3", "servoBRCalf"}, 
            v_joint_angles);

        return;
    }

    void update_servo_positions(std::vector<std::string> names, std::vector<double> angles){

        interfaces::msg::MotorPosition message;
        
        std::vector<uint16_t> motors;
        std::vector<double> pulses;
        
        trajectory_msgs::msg::JointTrajectory joint_message;
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;

        trajectory_point.time_from_start.sec = 1/50.0;

        // Loop over each servo that was provided
        for(uint16_t i = 0; i < names.size(); i++){

            // Format data for Gazebo joints. We are not simulating the entire linkage system so a calculated calf angle 
            // needs to be provided
            if(names[i].back() != '3'){
                joint_message.joint_names.push_back(names[i]);
                if(names[i].back() == '2')
                {
                    trajectory_point.positions.push_back(angles[i] * -1);
                }
                else {
                    trajectory_point.positions.push_back(angles[i]);
                }
                
            }

            double angle = angles[i];

            // Find the servo data based on the servo name
            auto servo = servo_map_.find(names[i]);

            // If the value doesn't exist in the servo map then skip the motor (Mostly for sim calf joint)
            if(servo == servo_map_.end()){
                continue;
            }

            const auto& servo_num = servo->second;
            MotorData servo_params = motor_parameters[servo_num];

            // Update the servo angle based on the parameters
            if(servo_params.inverted) angle = angle * -1.0;  // Servos on the opposite side of the robot move in the opposite direction
            
            angle += servo_params.offset;           // Add the calibration offset
            angle = std::clamp(angle, servo_params.min_angle, servo_params.max_angle); // Clamp the angle based on the max and min parameters
    
            motors.push_back(servo_num);
            pulses.push_back(rad_to_us(angle, servo_params.min_us, servo_params.max_us));
        }

        joint_message.points.push_back(trajectory_point);

        _jointPublisher->publish(joint_message);

        message.motor = motors;
        message.pulses = pulses;

        //RCLCPP_INFO(this->get_logger(), "Sending Message");

        _publisher->publish(message);
    }

    void resetServos(){
        for(uint16_t servo_num = 0; servo_num < servo_map_.size(); servo_num++)
        {
            // Second motor in each group should be centered at pi/4 instead of 0
            double angle = (servo_num - 1) % 4 == 0 ? M_PI_2 / 2.0 : 0.0;

            auto servo = servo_map_.find(servo_names[servo_num]);
            const auto& servo_name = servo->second;

            update_servo_positions(std::vector<std::string>{servo_name}, std::vector<double>{angle});

            rclcpp::sleep_for(std::chrono::milliseconds(250));
        }

    }

    void moveServos(const interfaces::msg::MotorTesting &msg){
        update_servo_positions(msg.motors, msg.angles);   
    }

    void updateBalanceCorrection(const interfaces::msg::BalanceCorrection &msg){
        gait_control.set_roll_correction(msg.roll_correction);
        gait_control.set_pitch_correction(msg.pitch_correction);
    }

    void inputHandler(const sensor_msgs::msg::Joy &msg){

        // Set the frequency and turn scale based on the appropriate joystick axis
        gait_control.set_frequency_scale(std::abs(msg.axes[1]));
        gait_control.set_turn_scale(msg.axes[3]);

        // get the primary and transition gaits
        gait* activePrimaryGait = gait_control.get_primary_gait();
        gait* activeTransitionGait = gait_control.get_transition_gait(); 

        // Handle Transitions
        if(msg.axes[1] > 0.2 && msg.axes[1] < 0.5) {
            if(activePrimaryGait == &idle && activeTransitionGait != &forwardWalk)
            {
                gait_control.queue_gait(&forwardWalk);
            }
        } 
        else if(msg.axes[1] > 0.5) {
            if(activePrimaryGait == &forwardWalk && activeTransitionGait != &forwardTrot)
            {
                gait_control.queue_gait(&forwardTrot);
            }
        }
        else if(msg.axes[1] < -0.2){
            if(activePrimaryGait == &idle && activeTransitionGait != &reverseWalk)
            {
                gait_control.queue_gait(&reverseWalk);
            }
        }
        else if(std::abs(msg.axes[1]) <= 0.2)
        {
            
            if(activePrimaryGait != &idle && activePrimaryGait != &rest && activeTransitionGait != &idle)
            {
                RCLCPP_INFO(this->get_logger(), "Primary gate != rest");
                gait_control.queue_gait(&idle);
            }    
        }

        if(msg.buttons[0] == 1 && buttonReset[0])
        {
            buttonReset[0] = false;

            interfaces::msg::MotorPosition msg;
            _resetBalancePublisher->publish(msg);

            gait_control.enableBalanceOffsets = !gait_control.enableBalanceOffsets;
        }
        else if(msg.buttons[0] == 0 && !buttonReset[0]){
            buttonReset[0] = true;
        }

        if(msg.buttons[1] == 1 && buttonReset[1]){
            if(activePrimaryGait == &rest) gait_control.queue_gait(&idle);
            else if(activePrimaryGait == &idle) gait_control.queue_gait(&rest);
            buttonReset[1] = false;
        }
        else if(msg.buttons[1] == 0 && !buttonReset[1]){
            buttonReset[1] = true;
        }
        
    }

    double rad_to_us(double rad, double min_us, double max_us) const {
        
        // Convert radians to pulse length in microseconds
        double t = (rad + (M_PI_2 - 0.174533)) / (M_PI - 0.349066);
        if(t < 0.0) t = 0.0;
        if(t > 1.0) t = 1.0;
        return min_us + t * (max_us - min_us);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();

    return 0;
}
    