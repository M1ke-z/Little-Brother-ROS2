#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "sensor_msgs/msg/imu.hpp"
#include "interfaces/msg/balance_correction.hpp"
#include "interfaces/msg/motor_position.hpp"
#include "interfaces/msg/motor_testing.hpp"

#include <chrono>

struct imu {
    int64_t gyroXTotal = 0;
    int64_t gyroYTotal = 0;
    int64_t gyroZTotal = 0;

    int64_t accelXTotal = 0;
    int64_t accelYTotal = 0;
    int64_t accelZTotal = 0;

    // The offset registers use a different scale than the readout so it needs to be converted
    const double gyroOffsetFactor =  0.25; 
    const double accelOffsetFactor = 0.125; 

    // The offset used to convert LSB to phyiscal values
    const double accelConversionFactor = 4.0 / std::pow(2, 16); // g/lsb
    const double gyroConversionFactor = (500.0 / std::pow(2, 16)) * (M_PI/180.0); // rad/(lsb*sec)

    const uint16_t targetCalibrationSamples = 1000;

    int16_t calibrationSamples = 0;
};

class balance : public rclcpp::Node {
public:
    balance() : Node("balance_node") {
        i2c_dev_ = this->declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
        i2c_addr_ = this->declare_parameter<int>("i2c_addr", 0x68);

        open_i2c();

        start_gy521();

        isCalibrating = true;

        previousTime = this->get_clock()->now();

        _subIMU = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Imu::SharedPtr msg){
                handle_imu_update(*msg);
            }
        );

        _subPIDParams = this->create_subscription<interfaces::msg::MotorTesting>(
            "/balance/pid_params", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting msg){
                RCLCPP_INFO(this->get_logger(), "Recieved Values: %.2f, %.2f", msg.angles[0], msg.angles[1]);
                this->proportional_constant = msg.angles[0];
                this->derivative_constant = msg.angles[1];
            }
        );

        _subResetBalance = this->create_subscription<interfaces::msg::MotorPosition>(
            "/balance/reset_balance", rclcpp::QoS(10),
            [this](interfaces::msg::MotorPosition msg){
                this->correctionRoll = 0.0;
                this->correctionPitch = 0.0;
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 400.0),
            [this]() {
                if(isCalibrating) run_calibration_cycle();
                else run_PID(); 
            }
        );

        _balanceCorrectionPublisher = this->create_publisher<interfaces::msg::BalanceCorrection>(
            "control/balanceCorrection", 10
        );

        RCLCPP_INFO(this->get_logger(), "Running balance node");
    }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subIMU;
        rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subPIDParams;
        rclcpp::Subscription<interfaces::msg::MotorPosition>::SharedPtr _subResetBalance;

        rclcpp::Publisher<interfaces::msg::BalanceCorrection>::SharedPtr _balanceCorrectionPublisher;
        
        rclcpp::Time previousTime;

        double currentRoll;
        double currentPitch;

        double currentRollVelocity;
        double currentPitchVelocity;

        double correctionRoll;
        double correctionPitch;

        const double gyroAccelRatio = 0.95;

        double proportional_constant = 0.8;
        double derivative_constant = 0.15;
        const double integral_constant = 0;

        std::string i2c_dev_;
        int i2c_addr_;

        int fd_{-1};

        bool isCalibrating = false;

        struct imu gy521;

        // GY521 Registers
        const uint8_t ACCEL_START = 0x3B;
        const uint8_t GYRO_START = 0x43;

        const uint8_t GYRO_CONFIG = 0x1B;
        const uint8_t ACCEL_CONFIG = 0x1C;

        const uint8_t POWER_MANAGEMENT = 0x6B;
        
        const uint8_t CONFIG = 0x1A;

        // MODE2 bits
        const uint8_t MODE2_OUTDRV = 0x04;

        void open_i2c() {
            // Open the linux i2c bus 
            fd_ = open(i2c_dev_.c_str(), O_RDWR);
            ioctl(fd_, I2C_SLAVE, i2c_addr_);

            // If the integer is < 0 then the i2c bus has already been opened
            if (fd_ < 0) throw std::runtime_error("Failed to open GY521 I2C device"); 

        }

        void handle_imu_update(const sensor_msgs::msg::Imu& imuData) {
            rclcpp::Time currentTime = this->get_clock()->now();

            double dt = (currentTime - previousTime).seconds();
            previousTime = currentTime;

            double accelRoll = std::atan2(
                imuData.linear_acceleration.y, 
                std::sqrt(std::pow(imuData.linear_acceleration.z, 2) + std::pow(imuData.linear_acceleration.x, 2))
            );
            double accelPitch = std::atan2(
                imuData.linear_acceleration.x, 
                std::sqrt(std::pow(imuData.linear_acceleration.z, 2) + std::pow(imuData.linear_acceleration.y, 2))
            );

            double gyroRoll = imuData.angular_velocity.x * dt;
            double gyroPitch = imuData.angular_velocity.y * dt;

            currentRollVelocity = imuData.angular_velocity.x;
            currentPitchVelocity = imuData.angular_velocity.y;

            currentRoll = gyroAccelRatio * (currentRoll + gyroRoll) + (1 - gyroAccelRatio) * accelRoll;
            currentPitch = gyroAccelRatio * (currentPitch + gyroPitch) + (1 - gyroAccelRatio) * accelPitch;

        }

        void run_PID() {

            std::array<int16_t, 6> gy521Values;

            gy521Values = i2c_read_values();

            // If the first three values are zero assume that the read failed.
            if (gy521Values[0] == 0 && gy521Values[0] == 0 && gy521Values[0] == 0) return;

            double gyroX = gy521Values[0] * gy521.gyroConversionFactor;
            double gyroY = gy521Values[1] * gy521.gyroConversionFactor;
            double gyroZ = gy521Values[2] * gy521.gyroConversionFactor;

            double accelX = gy521Values[3] * gy521.accelConversionFactor;
            double accelY = gy521Values[4] * gy521.accelConversionFactor;
            double accelZ = gy521Values[5] * gy521.accelConversionFactor;

            rclcpp::Time currentTime = this->get_clock()->now();

            double dt = (currentTime - previousTime).seconds();
            previousTime = currentTime;

            double accelRoll = std::atan2(
                accelY, 
                std::sqrt(std::pow(accelZ, 2) + std::pow(accelX, 2))
            );
            double accelPitch = std::atan2(
                accelX, 
                std::sqrt(std::pow(accelZ, 2) + std::pow(accelY, 2))
            );

            double gyroRoll = gyroX * dt;
            double gyroPitch = gyroY * dt;

            currentRollVelocity = gyroX;
            currentPitchVelocity = gyroY;

            currentRoll = gyroAccelRatio * (currentRoll + gyroRoll) + (1 - gyroAccelRatio) * accelRoll;
            currentPitch = gyroAccelRatio * (currentPitch + gyroPitch) + (1 - gyroAccelRatio) * accelPitch;

            // This PID script runs assuming that the target pitch and roll is 0
            double proportionalRoll = currentRoll * proportional_constant; 
            double derivativeRoll = currentRollVelocity * derivative_constant; 
            
            double proportionalPitch = currentPitch * proportional_constant; 
            double derivativePitch = currentPitchVelocity * derivative_constant; 

            correctionRoll = std::clamp(correctionRoll + proportionalRoll + derivativeRoll, -15.0, 15.0);
            correctionPitch = std::clamp(correctionPitch + proportionalPitch + derivativePitch, -15.0, 15.0);

            interfaces::msg::BalanceCorrection msg;

            msg.roll_correction = correctionRoll;
            msg.pitch_correction = correctionPitch;

            _balanceCorrectionPublisher->publish(msg);
        }

        void i2c_write(void* buf, uint8_t size) {
            if (write(fd_, buf, size) != size) throw std::runtime_error("I2C write failed");
        }

        void start_gy521(){
            uint8_t start_buf[2] = {POWER_MANAGEMENT, 0x00};
            uint8_t reset_buf[2] = {POWER_MANAGEMENT, 0x80};

            i2c_write(&reset_buf, 2);

            rclcpp::sleep_for(std::chrono::seconds(4));

            i2c_write(&start_buf, 2);
        }

        void run_calibration_cycle(){
            
            std::array<int16_t, 6> gy521Data = i2c_read_values();

            gy521.gyroXTotal += gy521Data[0];
            gy521.gyroYTotal += gy521Data[1];
            gy521.gyroZTotal += gy521Data[2];

            gy521.accelXTotal += gy521Data[3];
            gy521.accelYTotal += gy521Data[4];
            gy521.accelZTotal += gy521Data[5];

            gy521.calibrationSamples += 1;

            if(gy521.calibrationSamples == gy521.targetCalibrationSamples)
            {
                isCalibrating = false;

                // Acceleration offset registers have a value by default so that needs to be included in the offset.
                int16_t offsetAccelXCur = i2c_read_register_16(0x06);
                int16_t offsetAccelYCur = i2c_read_register_16(0x08);
                int16_t offsetAccelZCur = i2c_read_register_16(0x0A);

                int16_t offsetGyroX = std::floor((gy521.gyroXTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 
                int16_t offsetGyroY = std::floor((gy521.gyroYTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 
                int16_t offsetGyroZ = std::floor((gy521.gyroZTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 

                int16_t offsetAccelX = std::floor((gy521.accelXTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) * -1 + offsetAccelXCur; 
                int16_t offsetAccelY = std::floor((gy521.accelYTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) * -1 + offsetAccelYCur; 
                int16_t offsetAccelZ = std::floor((-16384 - gy521.accelZTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) + offsetAccelZCur; 

                // Bit 0 of the acceleration registers is used for temperature control and should not be changed.
                offsetAccelX = (offsetAccelX & ~1) | (1 & offsetAccelXCur);
                offsetAccelY = (offsetAccelY & ~1) | (1 & offsetAccelYCur);
                offsetAccelZ = (offsetAccelZ & ~1) | (1 & offsetAccelZCur);

                std::array<uint8_t, 7> accelOffsets = {
                    0x06, 
                    static_cast<uint8_t>(offsetAccelX >> 8), static_cast<uint8_t>(offsetAccelX & 0x00FF), 
                    static_cast<uint8_t>(offsetAccelY >> 8), static_cast<uint8_t>(offsetAccelY & 0x00FF),
                    static_cast<uint8_t>(offsetAccelZ >> 8), static_cast<uint8_t>(offsetAccelZ & 0x00FF)
                };

                std::array<uint8_t, 7> gyroOffsets = {
                    0x13, 
                    static_cast<uint8_t>(offsetGyroX >> 8), static_cast<uint8_t>(offsetGyroX & 0x00FF), 
                    static_cast<uint8_t>(offsetGyroY >> 8), static_cast<uint8_t>(offsetGyroY & 0x00FF),
                    static_cast<uint8_t>(offsetGyroZ >> 8), static_cast<uint8_t>(offsetGyroZ & 0x00FF)
                };

                i2c_write(&accelOffsets, 7);

                i2c_write(&gyroOffsets, 7);
            }
        }

        std::array<int16_t, 6> i2c_read_values() {
            std::array<uint8_t, 12> registerData;

            uint8_t gyroStartAddr = GYRO_START;
            uint8_t accelStartAddr = ACCEL_START;

            std::array<int16_t, 6> returnData = {0, 0, 0, 0, 0, 0};


            if (write(fd_, &gyroStartAddr, 1) != 1) return returnData; 
            if (read(fd_, &registerData, 6) != 6) return returnData; 

            if (write(fd_, &accelStartAddr, 1) != 1) return returnData; 
            if (read(fd_, &registerData[0] + 6, 6) != 6) return returnData; 

            

            for(uint16_t index = 0; index < registerData.size(); index += 2)
            {
                returnData[index/2] = (registerData[index] << 8 | registerData[index + 1]);
            }

            return returnData;
        }

        int16_t i2c_read_register_16(uint8_t addr){
            std::array<uint8_t, 2> registerData;

            if(write(fd_, &addr, 1) != 1) throw std::runtime_error("I2C reg select failed"); 
            if(read(fd_, &registerData, 2) != 2) throw std::runtime_error("I2C read failed");

            return registerData[0] << 8 | registerData[1];
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<balance>());
    rclcpp::shutdown();

    return 0;
}