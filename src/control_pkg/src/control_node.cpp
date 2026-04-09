#include <cmath>
#include <algorithm>

#include "control_pkg/control_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/int.hpp"
#include "interfaces/msg/double.hpp"
#include "interfaces/msg/motor_position.hpp"
#include "interfaces/msg/motor_testing.hpp"

#include "sensor_msgs/msg/joy.hpp"

class gait{
    public:
        gait(std::array<uint16_t, 4> phase_offsets, double frequency, std::array<std::array<double, 3>, 6> swing_control_points, std::array<std::array<double, 3>, 2> stance_control_points, std::array<double, 3> curve_offsets, double stance_ratio){
            phaseOffsets = offsets;
            gaitFrequency = frequency;
            swingControlPoints = swing_control_points;
            stanceControlPoints = stance_control_points;
            curveOffsets = curve_offsets;
            stanceRatio = stance_ratio;
        }

    private:
        std::array<uint16_t, 4> phaseOffsets;
        double gaitFrequency;

        std::array<double, 3> curveOffsets;

        std::array<std::array<double, 3>, 6> swingControlPoints;
        std::array<std::array<double, 3>, 2> stanceControlPoints; 

        double swingSwitchPhase = 180.0;
}

class global_gait_control {
    public:
        global_gait_control(){
            phaseClock = 0.0;
            gaitFrequency = 0.0;
            transitionProgress = 0.0;
            leftScale = 1.0;
            rightScale = 1.0;
            verticalScale = 1.0;

            inTransition = true;
        }

        void queue_gait(gait gait){
            if(primaryGait == null) {
                primaryGait = gait;
            }
            else if(!inTransition) {
                transitionGait = gait;
            }
        }

        std::array<double, 3> get_foot_position(uint16_t leg_num){

            double primaryPhase = std::fmod(phaseClock + primaryGait.phaseOffsets[leg_num], 360.0);

            std::array<double, 3> primary_position;

            if(phaseClock <= primaryGait.swingSwitchPhase) primary_position = this->get_curve_points(primaryGait.swingControlPoints, primaryGait.curveOffsets, primaryPhase / primaryGait.swingSwitchPhase);
            else primary_position = this->get_curve_points(primaryGait.stanceControlPoints, primaryGait.curveOffsets, (primaryPhase - primaryGait.swingSwitchPhase) / (360.0 - primaryGait.swingSwitchPhase));

            if(inTransition){
                double transitionPhase = std::fmod(phaseClock + transitionGait.phaseOffsets[leg_num], 360.0);

                std::array<double, 3> transition_position;
                
                if(phaseClock <= transitionGait.swingSwitchPhase) transition_position = this->get_curve_points(transitionGait.swingControlPoints, transitionGait.curveOffsets, transitionPhase / transitionGait.swingSwitchPhase);
                else transition_position = this->get_curve_points(transitionGait.stanceControlPoints, transitionGait.curveOffsets, (transitionPhase - transitionGait.swingSwitchPhase) / (360.0 - transitionGait.swingSwitchPhase));
                
                std::array<double, 3> transition_position = this->get_curve_points(transitionGait.swingControlPoints, transitionGait.curveOffsets, phaseClock);

                std::array<double, 3> blended_position = {
                    (1.0 - transitionProgress) * primary_position[0] + transitionProgress * transition_position[0],
                    (1.0 - transitionProgress) * primary_position[1] + transitionProgress * transition_position[1],
                    (1.0 - transitionProgress) * primary_position[2] + transitionProgress * transition_position[2]
                };

                return blended_position;
            }
            else {
                return primary_position;
            }
        }

        std::array<std::array<double, 3>, 4> runNextTick(){
            if(inTransition){
                transitionProgress += 1 / 50.0 * 4.0; // This should be based on the time since the last tick and the desired transition time

                if(transitionProgress >= 1.0){
                    primaryGait = transitionGait;
                    transitionGait = null;
                    inTransition = false;
                }
                else {
                    gaitFrequency = primaryGait.gaitFrequency + (transitionGait.gaitFrequency - primaryGait.gaitFrequency) * transitionProgress;
                }
            }

            phaseClock += std::fmod(360.0 * gaitFrequency / 50.0, 360.0); // This might cause a weird slowing effect. Look into this later

            std::array<std::array<double, 3>, 4> foot_positions;

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++){
                foot_positions[leg_num] = this->get_foot_position(leg_num);
            }

            return foot_positions;
        }
        
    private:
        double phaseClock;
        double gaitFrequency;
        double transitionProgress;
        double leftScale, rightScale, verticalScale;

        double stanceSwitchPrimary;
        double stanceSwitchTransition;

        bool inTransition;

        gait primaryGait;
        gait transitionGait;

        std::array<double, 3> get_curve_points(std::array<std::array<double, 3>, 6> bezierControlPoints, std::array<double, 3> curveOffsets, double t)
        {
            std::array<std::function<double(double)>, 6> bernstein_polynomials = {
                [](double t) {return std::pow((1.0-t), 5);},
                [](double t) {return 5 * std::pow((1.0-t), 4) * t;},
                [](double t) {return 10 * std::pow((1.0-t), 3) * pow(t, 2);},
                [](double t) {return 10 * std::pow((1.0-t), 5) * pow(t, 3);},
                [](double t) {return 5 * std::pow((1.0-t), 5) * pow(t, 3);},
                [](double t) {return std::pow((t), 5);}
            };

            std::array<double, 3> position = {0.0, 0.0, 0.0};

            for(uint16_t index = 0; index < points.size(); index++)
            {
                position[0] += bernstein_polynomials[index](t) * points[index][0];
                position[1] += bernstein_polynomials[index](t) * points[index][1];
                position[2] += bernstein_polynomials[index](t) * points[index][2];
            }  

            position[0] = position[0] + curveOffsets[0];
            position[1] = position[1] + curveOffsets[1];
            position[2] = position[2] + curveOffsets[2];

            return position;
        }
}

class control : public rclcpp::Node {  
public:
    control() : Node("control_node") {
        RCLCPP_INFO(this->get_logger(), "Control Node Started");
        this->declare_parameter("names", std::vector<std::string>{});
        this->declare_parameter("positions", std::vector<double>{});

        // These channels should be passed in as parameters rather than hardcoded. Look into this
        // FL1 - Front, Left, 1st Motor
        // 1st Motor - Shoulder Joint
        // 2nd Motor - Upper Joint
        // 3rd Motor - Lower Elbow Joint
        servo_map_ = {
            {"servoFL1", 0},
            {"servoFL2", 1},
            {"servoFL3", 2},
            {"servoFR1", 4},
            {"servoFR2", 5},
            {"servoFR3", 6},
            {"servoBR1", 8},
            {"servoBR2", 9},
            {"servoBR3", 10},
            {"servoBL1", 12},
            {"servoBL2", 13},
            {"servoBL3", 14}
        };

        // This is used in the event that all of the servos need to be updated and provides and easy way to iterate over them
        // The update servo function should really be modified to take a vector of integers instead of strings so that this is not needed
        servo_names = {"servoFL1", "servoFL2", "servoFL3", "servoFR1", "servoFR2", "servoFR3", "servoBR1", "servoBR2", "servoBR3", "servoBL1", "servoBL2", "servoBL3"};

        _sub = this->create_subscription<interfaces::msg::Int>(
            "control/walk", rclcpp::QoS(10),
            [this](interfaces::msg::Int::SharedPtr msg){
                walkGait(*msg);
            }
        );  

        _subController = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Joy::SharedPtr msg){
                inputHandler(*msg);
            }
        );

        _subTrot = this->create_subscription<interfaces::msg::Int>(
            "control/trot", rclcpp::QoS(10),
            [this](interfaces::msg::Int::SharedPtr msg){
                trotGait(*msg);
            }
        ); 

        _subMove = this->create_subscription<interfaces::msg::MotorTesting>(
            "control/moveServos", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting::SharedPtr msg){
                moveServos(*msg);
            }
        );  

        _subReset = this->create_subscription<interfaces::msg::Int>(
            "control/resetServos", rclcpp::QoS(10),
            [this](interfaces::msg::Int::SharedPtr msg){
                resetServos();
            }
        );  

        _publisher = this->create_publisher<interfaces::msg::MotorPosition>(
            "servo/command", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 50.0),
            [this]() { updateFeetPosition(); }
        );

        declare_motor_parameters(*this);
        motor_parameters = get_motor_parameters(*this);

        gait_control = new global_gait_control();

        idle = new gait(
            {0, 0, 0, 0}, 
            0.1, 
            {{
                {{18.0, 0.0, 0.0}}, 
                {{18.0, 0.0, 0.0}}, 
                {{18.0, 0.0, 0.0}}, 
                {{18.0, 0.0, 0.0}}, 
                {{18.0, 0.0, 0.0}}, 
                {{18.0, 0.0, 0.0}}
            }},
            {-20.0, -145.0, 0.0}
        )

        forwardTrot = new gait(
            {0, 180, 180, 0}, 
            2.0, 
            {{
                {{18.0, 0.0, 0.0}}, 
                {{36.0, 0.0, 0.0}}, 
                {{60.0, 70.0, 0.0}}, 
                {{-90.0, 70.0, 0.0}}, 
                {{-66.0, 0.0, 0.0}}, 
                {{-48.0, 0.0, 0.0}}
            }},
            {-20.0, -145.0, 0.0}
        )

        reverseTrot = new gait(
            {0, 180, 180, 0}, 
            2.0, 
            {{
                {{18.0, 0.0, 0.0}}, 
                {{36.0, 0.0, 0.0}}, 
                {{60.0, 70.0, 0.0}}, 
                {{-90.0, 70.0, 0.0}}, 
                {{-66.0, 0.0, 0.0}}, 
                {{-48.0, 0.0, 0.0}}
            }},
            {-20.0, -145.0, 0.0}
        )

    }

private:
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subController;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subTrot;
    rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subMove;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subReset;
    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, uint8_t> servo_map_;
    std::array<std::string, 12> servo_names;

    std::array<MotorData, 16> motor_parameters;

    bool isWalking = false;
    double gaitSpeed = 0.0;
    double turnDirection = 0.0;
    double leftTurnProgress = 0.0;
    double rightTurnProgress = 0.0;

    global_gait_control gait_control;

    gait idle;
    gait forwardTrot;
    gait reverseTrot;

    // Declare constant linkage lengths and angles (rad). See MZ for joint defenitions 
    const std::array<double, 8> linkLen = {24.0, 20.0, 38.0, 25.0, 95.0, 122.7};
    const std::array<double, 2> linkAng = {2.35619};

    const std::array<double, 2> motorPos = {-20, -20};

    double invCosLaw(double sideA, double sideB, double sideC) {
        // This function returns angle C in radians
        // sideA and sideB are interchangable 
        return std::acos((std::pow(sideC, 2) - std::pow(sideA, 2) - std::pow(sideB, 2))/(-2 * sideA * sideB));
    }

    std::array<double, 3> calcJointAngles(std::array<double, 3> footPosition){
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

        double motor1Ang = std::atan(std::abs(footPosition[1]+linkLen[5]*std::sin(calcAng[2]))/std::abs(footPosition[0]+linkLen[5]*std::cos(calcAng[2])));

        return std::array<double, 3> {0, motor1Ang, calcAng[6]};
    }

    void updateFeetPosition() {
        std::array<std::array<double, 3>, 4> footPositions = runNextTick();

        std::vector<double> v_joint_angles = {};

        std::array<double, 3> joint_angles = calcJointAngles(foot_position);
        
        v_joint_angles.insert(v_joint_angles.end(), joint_angles.begin(), joint_angles.end());

        update_servo_positions(std::vector<std::string>{
            "servoFL1", "servoFL2", "servoFL3", 
            "servoFR1", "servoFR2", "servoFR3", 
            "servoBL1", "servoBL2", "servoBL3", 
            "servoBR1", "servoBR2", "servoBR3"}, 
            v_joint_angles);
    }

    void update_servo_positions(std::vector<std::string> names, std::vector<double> angles){
        // RCLCPP_INFO(this->get_logger(), "Started Position Update - Control Node");

        interfaces::msg::MotorPosition message;
        
        std::vector<uint16_t> motors;
        std::vector<double> pulses;
        
        // Loop over each servo that was provided
        for(uint16_t i = 0; i < names.size(); i++){
            double angle = angles[i];
            // RCLCPP_INFO(this->get_logger(), "%.2f", angle);
            // Find the servo data based on the servo name
            auto servo = servo_map_.find(names[i]);
            const auto& servo_num = servo->second;
            MotorData servo_params = motor_parameters[servo_num];

            // Update the servo angle based on the parameters
            if(servo_params.inverted) angle = angle * -1.0;  // Servos on the opposite side of the robot move in the opposite direction
            // RCLCPP_INFO(this->get_logger(), "%.2f", angle);
            angle += servo_params.offset;           // Add the calibration offset
            // RCLCPP_INFO(this->get_logger(), "%.2f", angle);
            angle = std::clamp(angle, servo_params.min_angle, servo_params.max_angle); // Clamp the angle based on the max and min parameters
            // RCLCPP_INFO(this->get_logger(), "%.2f", angle);
            motors.push_back(servo_num);
            pulses.push_back(rad_to_us(angle, servo_params.min_us, servo_params.max_us));
        }

        message.motor = motors;
        message.pulses = pulses;

        // RCLCPP_INFO(this->get_logger(), "%d, %.2f", message.motor[0], message.pulses[0]);

        _publisher->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Message Published");
    }

    void resetServos(){
        for(uint16_t servo_num = 0; servo_num < servo_map_.size(); servo_num++)
        {
            RCLCPP_INFO(this->get_logger(), "%d", servo_num);

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

    void walkGait(const interfaces::msg::Int &msg){
        // Left side (0, 2) -> shift right (-0.15)
        // Right side (1, 3) -> shift left (0.15)
        std::array<uint16_t, 4> leg_cycles = {0, 90, 60, 30};

        double cycle_ticks = 120.0;

        uint16_t next_leg = 0;

        const double max_leg_tilt = 0.15;
        const double leg_tilt_increment = (max_leg_tilt*2) / (cycle_ticks/8.0);
        double leg_tilt = -max_leg_tilt; // Assuming we are starting with a left side leg

        const double fw_time_factor = (70.0-(-15.0))/(cycle_ticks/8);
        const double bw_time_factor = (70.0-(-15.0))/((cycle_ticks/8)*7);

        isWalking = true;
        
        for(uint16_t i = 0; i < cycle_ticks*msg.val; i++)
        {
            std::vector<double> v_joint_angles = {};
            
            if(next_leg == 1 || next_leg == 3) leg_tilt = std::min(max_leg_tilt, leg_tilt + leg_tilt_increment);
            else if(next_leg == 0 || next_leg == 2) leg_tilt = std::max(-max_leg_tilt, leg_tilt - leg_tilt_increment);

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++)
            {
                if(leg_cycles[leg_num] >= (cycle_ticks/8.0 * 7)) next_leg = leg_num;

                std::array<double, 3> foot_position = {0.0, 0.0, 0.0};
                double t = 0;

                if(leg_cycles[leg_num] < (cycle_ticks/8)){
                    t = -15 + leg_cycles[leg_num] * fw_time_factor;
                    foot_position = {-t, -0.0221*std::pow(t-27.5, 2)-120.0, 0};
                } 
                else {
                    double y = -160;
                    t = 70 - (leg_cycles[leg_num] - (cycle_ticks/4)) * bw_time_factor;
                    foot_position = {-t, y, 0};
                }

                std::array<double, 3> joint_angles = calcJointAngles(foot_position);
                joint_angles[0] = leg_tilt;

                double angle_0 = std::atan(std::abs(foot_position[1]) / std::abs(foot_position[0]));

                if(foot_position[0] > 0) angle_0 = M_PI - angle_0;

                if(leg_num == 0) RCLCPP_INFO(this->get_logger(), "Cycles: %d, Motor2: %.2f, x: %.2f, y: %.2f, angle_0: %.2f", leg_cycles[0], joint_angles[1], foot_position[0], foot_position[1], angle_0);

                v_joint_angles.insert(v_joint_angles.end(), joint_angles.begin(), joint_angles.end());

                if(leg_cycles[leg_num] >= static_cast<uint16_t>(cycle_ticks-1)) leg_cycles[leg_num] = 0;
                else leg_cycles[leg_num]++;
            }

            //RCLCPP_INFO(this->get_logger(), "Tilt: %.2f", leg_tilt);

            update_servo_positions(std::vector<std::string>{
                "servoFL1", "servoFL2", "servoFL3", 
                "servoFR1", "servoFR2", "servoFR3", 
                "servoBL1", "servoBL2", "servoBL3", 
                "servoBR1", "servoBR2", "servoBR3"}, 
                v_joint_angles);

            

            rclcpp::sleep_for(std::chrono::milliseconds(1000/50));
        }
        isWalking = false;
    }

    void trotGait(const interfaces::msg::Int &msg){
        // Left side (0, 2) -> shift right (-0.15)
        // Right side (1, 3) -> shift left (0.15)
        
        double cycle_ticks = 50.0 + std::round(50.0 * (1.0 - gaitSpeed));

        RCLCPP_INFO(this->get_logger(), "Ticks: %f", cycle_ticks);

        std::array<uint16_t, 4> leg_cycles = {0, static_cast<uint16_t>(cycle_ticks/2.0), static_cast<uint16_t>(cycle_ticks/2.0), 0};

        RCLCPP_INFO(this->get_logger(), "Cycles: %d, %d, %d, %d", leg_cycles[0], leg_cycles[1], leg_cycles[2], leg_cycles[3]);

        isWalking = true;
        
        for(uint16_t i = 0; i < cycle_ticks*msg.val; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Right: %.2f, Left %.2f", rightTurnProgress, leftTurnProgress);
            if(turnDirection > 0.0){
                if(rightTurnProgress < turnDirection * 10) rightTurnProgress += 0.5;
            }
            else {
                if(rightTurnProgress > 0.0){
                    rightTurnProgress -= 0.5;
                    std::max(rightTurnProgress, 0.0);
                }
            }

            if(turnDirection < 0.0)
            {
                if(leftTurnProgress < std::abs(turnDirection * 10)) leftTurnProgress += 0.5;
            }
            else {
                if(leftTurnProgress > 0.0){
                    leftTurnProgress -= 0.5;
                    std::max(leftTurnProgress, 0.0);
                }
            }

            std::vector<double> v_joint_angles = {};

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++)
            {
                std::array<double, 3> foot_position = {0.0, 0.0, 0.0};
                double t = 0;

                if(leg_cycles[leg_num] < (cycle_ticks/2)){
                    std::array<double, 2> points;
                    if(leg_num == 0 || leg_num == 2){
                        points = get_curve_points(leg_cycles[leg_num] / (cycle_ticks/2), -leftTurnProgress, leftTurnProgress);
                    } 
                    else{
                        points = get_curve_points(leg_cycles[leg_num] / (cycle_ticks/2), -rightTurnProgress, rightTurnProgress);
                    } 
                    foot_position = {points[0], points[1], 0};
                } 
                else {
                    double y = -145;
                    if(leg_num == 0 || leg_num == 2){
                        t = -68 + leftTurnProgress + (66 - leftTurnProgress * 2)*(leg_cycles[leg_num]/(cycle_ticks/2)-1); 
                    } 
                    else{
                        t = -68 + rightTurnProgress + (66 - rightTurnProgress * 2)*(leg_cycles[leg_num]/(cycle_ticks/2)-1); 
                    } 
                    foot_position = {t, y, 0};
                }
                
                std::array<double, 3> joint_angles = calcJointAngles(foot_position);

                //if(leg_num == 0) RCLCPP_INFO(this->get_logger(), "Cycles: %d, x: %.2f, y: %.2f", leg_cycles[0], foot_position[0], foot_position[1]);

                v_joint_angles.insert(v_joint_angles.end(), joint_angles.begin(), joint_angles.end());

                if(leg_cycles[leg_num] >= static_cast<uint16_t>(cycle_ticks-1)) leg_cycles[leg_num] = 0;
                else leg_cycles[leg_num]++;
            }

            //RCLCPP_INFO(this->get_logger(), "Tilt: %.2f", leg_tilt);

            update_servo_positions(std::vector<std::string>{
                "servoFL1", "servoFL2", "servoFL3",
                "servoFR1", "servoFR2", "servoFR3", 
                "servoBL1", "servoBL2", "servoBL3", 
                "servoBR1", "servoBR2", "servoBR3"}, 
                v_joint_angles);

            rclcpp::sleep_for(std::chrono::milliseconds(1000/50));
        }

        isWalking = false;
    }

    void trotReverseGait(const interfaces::msg::Int &msg){
        // Left side (0, 2) -> shift right (-0.15)
        // Right side (1, 3) -> shift left (0.15)
        
        double cycle_ticks = 70.0;

        RCLCPP_INFO(this->get_logger(), "Ticks: %f", cycle_ticks);

        std::array<uint16_t, 4> leg_cycles = {0, static_cast<uint16_t>(cycle_ticks/2.0), static_cast<uint16_t>(cycle_ticks/2.0), 0};

        RCLCPP_INFO(this->get_logger(), "Cycles: %d, %d, %d, %d", leg_cycles[0], leg_cycles[1], leg_cycles[2], leg_cycles[3]);

        isWalking = true;
        
        for(uint16_t i = cycle_ticks*msg.val; i > 0; i--)
        {
            std::vector<double> v_joint_angles = {};

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++)
            {
                std::array<double, 3> foot_position = {0.0, 0.0, 0.0};
                double t = 0;

                if(leg_cycles[leg_num] < (cycle_ticks/2)){
                    std::array<double, 2> points = get_curve_points(leg_cycles[leg_num] / (cycle_ticks/2), 0.0, 0.0);
                    foot_position = {points[0], points[1], 0};
                } 
                else {
                    double y = -145;
                    t = -68 + 66*(leg_cycles[leg_num]/(cycle_ticks/2)-1);  
                    foot_position = {t, y, 0};
                }
                
                std::array<double, 3> joint_angles = calcJointAngles(foot_position);

                if(leg_num == 0) RCLCPP_INFO(this->get_logger(), "Cycles: %d, x: %.2f, y: %.2f", leg_cycles[0], foot_position[0], foot_position[1]);

                v_joint_angles.insert(v_joint_angles.end(), joint_angles.begin(), joint_angles.end());

                if(leg_cycles[leg_num] <= 0) leg_cycles[leg_num] = static_cast<uint16_t>(cycle_ticks-1);
                else leg_cycles[leg_num]--;
            }

            //RCLCPP_INFO(this->get_logger(), "Tilt: %.2f", leg_tilt);

            update_servo_positions(std::vector<std::string>{
                "servoFL1", "servoFL2", "servoFL3",
                "servoFR1", "servoFR2", "servoFR3", 
                "servoBL1", "servoBL2", "servoBL3", 
                "servoBR1", "servoBR2", "servoBR3"}, 
                v_joint_angles);

            rclcpp::sleep_for(std::chrono::milliseconds(1000/50));
        }

        isWalking = false;
    }

    void inputHandler(const sensor_msgs::msg::Joy &msg){
        if(msg.axes[1] > 0.2 && !isWalking) {
            if(msg.axes[1] > 0.8) gaitSpeed = 1;
            else gaitSpeed = msg.axes[1] / 0.8; 
            
            interfaces::msg::Int msg;
            msg.val = 1;
            trotGait(msg);
        }
        else if(msg.axes[1] < -0.2 && !isWalking)
        {
            RCLCPP_INFO(this->get_logger(), "Run Reverse Walk Cycle");
            interfaces::msg::Int msg;
            msg.val = 1;
            trotReverseGait(msg);
        }
        else {

        }

        RCLCPP_INFO(this->get_logger(), "Turn Axis: %.2f", msg.axes[3]);
        if(std::abs(msg.axes[3]) > 0.2){
            turnDirection = msg.axes[3] * -1;
        }
        else {
            turnDirection = 0;
        }
    }

    double rad_to_us(double rad, double min_us, double max_us) const {
        // Example: map [-pi/2, pi/2] to [min_us_, max_us_]

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
    