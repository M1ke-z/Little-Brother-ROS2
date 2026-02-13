#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <interfaces/msg/MovementData.hpp>

#include <cmath>

// At some point these might need to be their own nodes. Additionally setting custom pulses will be required
struct MotorData {
    double maxAngle;
    double minAngle;
};

// Need to turn publisher into joint angle publisher

class control : public rclcpp::Node {  
public:
    control() : Node("control_node") {
        std::cout << "Control Node Started" << std::endl;

        _sub = this->create_subscription<interfaces::msg::MovementData>(
            "control/travelCommand", rclcpp::QoS(10),
            [this](interfaces::msg::MovementData::SharedPtr msg){moveServo(*msg);}
        );

        _publisher = this->create_publisher<sensor_msgs::msg::JointState>(
            "servo/command", 10
        );
    }

private:
    rclcpp::Subscription<interfaces::msg::MovementData>::SharedPtr _sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;

    // These values should be a parameter that can be adjusted outside of the script
    const std::array<MotorData, 3> servoMotorData = {{{M_PI_2, -M_PI_2}, {0.5, -0.95}, {M_PI_2, 0.1}}};

    // Declare constant linkage lengths and angles (rad). See MZ for joint defenitions 
    const std::array<double, 8> linkLen = {24, 20, 35, 25, 95, 115, 154.3, 20};
    const std::array<double, 2> linkAng = {0.15533, 1.67097};

    const std::array<double, 2> motorPos = {-20, -20};

    double invCosLaw(double sideA, double sideB, double sideC)
    {
        // This function returns angle C in radians
        // sideA and sideB are interchangable 
        return std::acos((std::pow(sideC, 2) - std::pow(sideA, 2) - std::pow(sideB, 2))/(-2 * sideA * sideB));
    }

    std::array<double, 3> getJointAngles(std::array<double, 3> footPosition){
        std::array<double, 3> clampedAngles = {};

        std::array<double, 3> motorAngles = calcJointAngles(footPosition);

        clampedAngles[0] = std::clamp(motorAngles[0], servoMotorData[0].maxAngle, servoMotorData[0].minAngle);
        clampedAngles[1] = std::clamp(motorAngles[1], servoMotorData[1].maxAngle, servoMotorData[1].minAngle);
        clampedAngles[2] = std::clamp(motorAngles[2], servoMotorData[2].maxAngle, servoMotorData[2].minAngle);

        for(uint8_t index = 0; index < 3; index++) {
            if(clampedAngles[index] != motorAngles[index]) std::cout << "WARNING: Servo " << index << " Outside of Clamped Values";
        } 

        return clampedAngles;
    }

    std::array<double, 3> calcJointAngles(std::array<double, 3> footPosition){
        // Inverse kinematics to calculate the joint angles requried to reach a certian position

        // Declare arrays to hold calculated intermediate values 
        std::array<double, 3> calcLen = {};
        std::array<double, 10> calcAng = {};

        // Calculate joint angles 
        calcLen[0] = std::sqrt(std::pow(footPosition[0], 2) + std::pow(footPosition[1], 2));
        calcAng[3] = invCosLaw(linkLen[4], linkLen[6], calcLen[0]);
        calcAng[5] = std::asin((linkLen[6]/calcLen[0])*std::sin(calcAng[3]));
        calcAng[6] = std::atan(footPosition[0]/footPosition[1]);
        calcAng[0] = M_PI_2 - calcAng[5] + calcAng[6];
        calcAng[2] = calcAng[3] - calcAng[0] - linkAng[0];
        
        double x1 = linkLen[4]*std::cos(calcAng[0]) + linkLen[7]*std::cos(calcAng[2]);
        double y1 = -linkLen[4]*std::sin(calcAng[0]) + linkLen[7]*std::sin(calcAng[2]);
        
        calcLen[1] = std::sqrt(std::pow(x1, 2) + std::pow(y1, 2));
        
        calcAng[7] = std::atan(std::abs(y1/x1));
        calcAng[1] = M_PI_2 - invCosLaw(linkLen[3], calcLen[1], linkLen[5]);
        calcAng[8] = linkAng[1] - calcAng[1];
        
        double x2 = -linkLen[2] * std::sin(calcAng[8]) - motorPos[0];
        double y2 =  linkLen[2] * std::cos(calcAng[8]) - motorPos[1];
        
        calcLen[2] = std::sqrt(std::pow(x2, 2) + std::pow(y2, 2));
        
        calcAng[9] = invCosLaw(linkLen[0], std::pow(calcLen[2], 2), linkLen[1]);
        calcAng[4] = std::atan(std::abs(y2/x2)); - calcAng[9];

        // Return an array with the values of 0, angle 0, and angle 4. (Motors 1, 2, 3)
        return std::array<double, 3> {0, calcAng[0] - M_PI_2, calcAng[4]};
    }

    void moveServo(const std_msgs::msg::Float64 &msg) {
        double rad_per_s = M_PI / msg.data;
        double rad_per_update = rad_per_s / 50;
        std::cout << "Recieved Message" << std::endl;
        double current_angle = -M_PI_2;

        sensor_msgs::msg::JointState message;

        message.name = {"servoFL1"};
        message.position = {current_angle};
        _publisher->publish(message);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();

    return 0;
}