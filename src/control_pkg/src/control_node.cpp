#include <cmath>
#include <algorithm>

#include "control_pkg/control_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/int.hpp"
#include "interfaces/msg/double.hpp"
#include "interfaces/msg/motor_position.hpp"
#include "interfaces/msg/motor_testing.hpp"

#include "sensor_msgs/msg/joy.hpp"

// Need to turn publisher into joint angle publisher

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
                walkGait(*msg);
            }
        )

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

        declare_motor_parameters(*this);
        motor_parameters = get_motor_parameters(*this);

        //this->declare_parameter<double>("time_val", 40.0);
    }

private:
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _sub;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subTrot;
    rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subMove;
    rclcpp::Subscription<interfaces::msg::Int>::SharedPtr _subReset;
    rclcpp::Publisher<interfaces::msg::MotorPosition>::SharedPtr _publisher;

    std::unordered_map<std::string, uint8_t> servo_map_;
    std::array<std::string, 12> servo_names;

    std::array<MotorData, 16> motor_parameters;

    // Declare constant linkage lengths and angles (rad). See MZ for joint defenitions 
    const std::array<double, 8> linkLen = {24.0, 20.0, 38.0, 25.0, 95.0, 122.7};
    const std::array<double, 2> linkAng = {2.35619};

    const std::array<double, 2> motorPos = {-20, -20};

    double invCosLaw(double sideA, double sideB, double sideC)
    {
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


        // // Calculate joint angles 
        // calcLen[0] = std::sqrt(std::pow(footPosition[0], 2) + std::pow(footPosition[1], 2));
        // calcAng[3] = invCosLaw(linkLen[4], linkLen[6], calcLen[0]);
        // calcAng[5] = std::asin((linkLen[6]/calcLen[0])*std::sin(calcAng[3]));
        // calcAng[6] = std::atan(footPosition[0]/footPosition[1]);
        // calcAng[0] = M_PI_2 - calcAng[5] + calcAng[6];
        // calcAng[2] = calcAng[3] - calcAng[0] - linkAng[0];
        
        // double x1 = linkLen[4]*std::cos(calcAng[0]) + linkLen[7]*std::cos(calcAng[2]);
        // double y1 = -linkLen[4]*std::sin(calcAng[0]) + linkLen[7]*std::sin(calcAng[2]);
        
        // calcLen[1] = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
        
        // calcAng[7] = std::atan(std::abs(y1/x1));
        // calcAng[1] = M_PI_2 - invCosLaw(linkLen[3], calcLen[1], linkLen[5]) + calcAng[7];
        // // RCLCPP_INFO(this->get_logger(), "calcAng1 invCos: %.2f", invCosLaw(linkLen[3], calcLen[1], linkLen[5]));
        // calcAng[8] = linkAng[1] - calcAng[1];

        // // RCLCPP_INFO(this->get_logger(), "linkAng1: %.2f, calcAng1: %.2f", linkAng[1], calcAng[1]);
        
        // double x2 = -linkLen[2] * std::sin(calcAng[8]) - motorPos[0];
        // double y2 =  linkLen[2] * std::cos(calcAng[8]) - motorPos[1];

        // // RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Angle: 8 %.2f", x2, y2, calcAng[8]);
        
        // calcLen[2] = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
        
        // calcAng[9] = invCosLaw(linkLen[0], calcLen[2], linkLen[1]);
        // calcAng[4] = std::atan(std::abs(y2/x2)) - calcAng[9];
        // // RCLCPP_INFO(this->get_logger(), "Tan: %.2f, Angle 9: %.2f, Angle 4: %.2f", std::atan(std::abs(y2/x2)), calcAng[9], calcAng[4]);
        // // Return an array with the values of 0, angle 0, and angle 4. (Motors 1, 2, 3)

        // // RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f - Length: 0: %.2f 1: %.2f - Angle: 0: %.2f, 1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f, 5: %.2f, 6: %.2f, 7: %.2f, 8: %.2f, 9: %.2f", footPosition[0], footPosition[1], calcLen[0], calcLen[1], calcAng[0], calcAng[1], calcAng[2], calcAng[3], calcAng[4], calcAng[5], calcAng[6], calcAng[7], calcAng[8], calcAng[9]);

        return std::array<double, 3> {0, motor1Ang, calcAng[6]};
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
        
        for(uint16_t i = 0; i < cycle_ticks*msg.val; i++)
        {
            std::vector<double> v_joint_angles = {};
            
            if(next_leg == 1 || next_leg == 3) leg_tilt = std::min(max_leg_tilt, leg_tilt + leg_tilt_increment);
            else if(next_leg == 0 || next_leg == 2) leg_tilt = std::max(-max_leg_tilt, leg_tilt - leg_tilt_increment);

            //if(next_leg != -1 && leg_cycles[next_leg] <= cycle_ticks/4.0) next_leg = -1;

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
                    // if(leg_cycles[leg_num] > 45 && leg_cycles[leg_num] < 75 && (leg_num == 3 || leg_num == 1))
                    // {
                    //     y = -160;
                    // }
                    // else if(leg_cycles[leg_num] > 165 && leg_cycles[leg_num] < 195 && (leg_num == 2 || leg_num == 0))
                    // {
                    //     y = -160;
                    // }
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
    }

    std::array<double, 2> get_curve_points(double t){
        std::array<std::array<int16_t, 2>, 6> points = {{{{18, 0}}, {{36, 0}}, {{60, 70}}, {{-90, 70}}, {{-66, 0}}, {{-48, 0}}}};

        std::array<std::function<double(double)>, 6> bernstein_polynomials = {
            [](double t) {return std::pow((1-t), 5);},
            [](double t) {return 5 * std::pow((1-t), 4) * t;},
            [](double t) {return 10 * std::pow((1-t), 3) * pow(t, 2);},
            [](double t) {return 10 * std::pow((1-t), 5) * pow(t, 3);},
            [](double t) {return 5 * std::pow((1-t), 5) * pow(t, 3);},
            [](double t) {return std::pow((t), 5);}
        };

        std::array<double, 2> position = {0.0, 0.0};

        for(uint16_t index = 0; index < points.size(); index++)
        {
            position[0] += bernstein_polynomials[index](t) * points[index][0];
            position[1] += bernstein_polynomials[index](t) * points[index][1];
        }  

        position[0] = position[0] - 20.0;
        position[1] = position[1] - 145.0;


        return position;
    }

    void trotGait(const interfaces::msg::Int &msg){
        // Left side (0, 2) -> shift right (-0.15)
        // Right side (1, 3) -> shift left (0.15)
        std::array<uint16_t, 4> leg_cycles = {0, 25, 25, 0};

        double cycle_ticks = 50.0;

        const double time_factor = 1/cycle_ticks;
        
        for(uint16_t i = 0; i < cycle_ticks*msg.val; i++)
        {
            std::vector<double> v_joint_angles = {};

            for(uint16_t leg_num = 0; leg_num < 4; leg_num++)
            {
                std::array<double, 3> foot_position = {0.0, 0.0, 0.0};
                double t = 0;

                if(leg_cycles[leg_num] < (cycle_ticks/2)){
                    std::array<double, 2> points = get_curve_points(leg_cycles[leg_num] / (cycle_ticks/2));
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
    