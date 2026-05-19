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

// The IMU struct contains all of the information needed to calibrate the IMU and convert its register data
struct imu {
    // gryoTotal and accelTotal variables store the summed values that will be used to calculate the average value 
    // for calibration
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

    const uint16_t targetCalibrationSamples = 1000; // The number of samples that should be taken for the average

    int16_t calibrationSamples = 0; // The number of calibration samples that have been taken so far
};

class balance : public rclcpp::Node {
public:
    balance() : Node("balance_node") {

        // Save the path for opening an I2C connection and the address of the IMU
        i2cDev = this->declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
        i2cAddr = this->declare_parameter<int>("i2c_addr", 0x68);

        // Open the connection
        open_i2c();

        // Run the startup steps for the IMU
        start_gy521();

        isCalibrating = true;

        // Set a default value for previous time so the first calibration cycle doesn't error out
        previousTime = this->get_clock()->now();

        // Define all of the subscriptions, publishers, and timers

        // This subscription is used for testing when the IMU data is being provided by Gazebo
        _subIMU = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Imu::SharedPtr msg){
                handle_imu_update(*msg);
            }
        );

        // This is used for calibrating the PD controller for the IMU 
        _subPIDParams = this->create_subscription<interfaces::msg::MotorTesting>(
            "/balance/pid_params", rclcpp::QoS(10),
            [this](interfaces::msg::MotorTesting msg){
                this->proportional_constant = msg.angles[0];
                this->derivative_constant = msg.angles[1];
            }
        );

        // Reset balance is called when stabilization is being enabled to reset the values to prevent the robot from
        // jumping
        _subResetBalance = this->create_subscription<interfaces::msg::MotorPosition>(
            "/balance/reset_balance", rclcpp::QoS(10),
            [this](interfaces::msg::MotorPosition msg){
                this->correctionRoll = 0.0;
                this->correctionPitch = 0.0;
            }
        );

        // This timer is for getting updated IMU values a running the PD loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 400.0),
            [this]() {
                // If we are in calibration mode the values will be handled by the calibration function
                if(isCalibrating) run_calibration_cycle();
                // Otherwise updated values should be handled by the PD controller
                else run_PID(); 
            }
        );

        // This publisher is used to send updated roll and pitch offsets to the control node
        _balanceCorrectionPublisher = this->create_publisher<interfaces::msg::BalanceCorrection>(
            "control/balanceCorrection", 10
        );

        RCLCPP_INFO(this->get_logger(), "Running balance node");
    }

    private:
        // Define the subscription, publisher, and timer variables. Their uses are described above.
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subIMU;
        rclcpp::Subscription<interfaces::msg::MotorTesting>::SharedPtr _subPIDParams;
        rclcpp::Subscription<interfaces::msg::MotorPosition>::SharedPtr _subResetBalance;

        rclcpp::Publisher<interfaces::msg::BalanceCorrection>::SharedPtr _balanceCorrectionPublisher;
        
        rclcpp::Time previousTime; // Used for checking when the last update cycle was run

        // Used for storing the roll and pitch of the robot calculated from the IMU data
        double currentRoll;
        double currentPitch;

        // Current values from gyroscope
        double currentRollVelocity;
        double currentPitchVelocity;

        // Offset correction values
        double correctionRoll;
        double correctionPitch;

        // Used for the complimentary filter. Shows how much the gyroscope data should be weighted compared to the 
        // angle from the accelerometer
        const double gyroAccelRatio = 0.95;

        // PID Tuning parameters
        double proportional_constant = 0.8;
        double derivative_constant = 0.15;
        // Integral is not being used and has not been implemented yet. It is not needed right now, I just added
        // it for completeness
        const double integral_constant = 0; 

        // I2C data
        std::string i2cDev;
        int i2cAddr;

        // File descriptor for the I2C line
        int fd_{-1};

        bool isCalibrating = false;

        struct imu gy521;

        // GY521 Registers
        // ToDo: These should probably be moved into the gy521 struct
        const uint8_t ACCEL_START = 0x3B;
        const uint8_t GYRO_START = 0x43;

        const uint8_t GYRO_CONFIG = 0x1B;
        const uint8_t ACCEL_CONFIG = 0x1C;

        const uint8_t POWER_MANAGEMENT = 0x6B;
        
        const uint8_t CONFIG = 0x1A;

        // MODE2 bits
        const uint8_t MODE2_OUTDRV = 0x04;

        void open_i2c() {
            /* ------
                Purpose:
                    Opens the I2C bus for the GY521
                Parameters:
                    None
                Return 
                    None
            ------ */ 

            // Open the linux i2c bus 
            fd_ = open(i2cDev.c_str(), O_RDWR);
            ioctl(fd_, I2C_SLAVE, i2cAddr);

            // If the integer is < 0 then the i2c bus has already been opened
            if (fd_ < 0) throw std::runtime_error("Failed to open GY521 I2C device"); 

        }

        void handle_imu_update(const sensor_msgs::msg::Imu& imuData) {
            /* ------
                Purpose:
                    For Gazebo simulation use only. Takes IMU data and calculates the pitch and roll
                Parameters:
                    const sensor_msgs::msg::Imu& imuData - IMU data
                Return 
                    None
            ------ */ 

            rclcpp::Time currentTime = this->get_clock()->now();

            // Get the change in time
            double dt = (currentTime - previousTime).seconds();
            previousTime = currentTime;

            // Calculate the roll and pitch from the accelerometer data
            double accelRoll = std::atan2(
                imuData.linear_acceleration.y, 
                std::sqrt(std::pow(imuData.linear_acceleration.z, 2) + std::pow(imuData.linear_acceleration.x, 2))
            );
            double accelPitch = std::atan2(
                imuData.linear_acceleration.x, 
                std::sqrt(std::pow(imuData.linear_acceleration.z, 2) + std::pow(imuData.linear_acceleration.y, 2))
            );

            // Calculate the change in angle. This will be used in the complementary filter
            double gyroRoll = imuData.angular_velocity.x * dt;
            double gyroPitch = imuData.angular_velocity.y * dt;

            currentRollVelocity = imuData.angular_velocity.x;
            currentPitchVelocity = imuData.angular_velocity.y;

            // Using a complementary filter calculate the roll and pitch of the robot
            currentRoll = gyroAccelRatio * (currentRoll + gyroRoll) + (1 - gyroAccelRatio) * accelRoll;
            currentPitch = gyroAccelRatio * (currentPitch + gyroPitch) + (1 - gyroAccelRatio) * accelPitch;

        }

        void run_PID() {
            /* ------
                Purpose:
                    Read the IMU data over I2C and update the foot offset values
                Parameters:
                    None
                Return 
                    None
            ------ */ 

            std::array<int16_t, 6> gy521Values;

            // Get the 3 gyroscope values and 3 accelerometer values from the GY521 registers
            gy521Values = i2c_read_values();

            // If the first three values are zero assume that the read failed.
            if (gy521Values[0] == 0 && gy521Values[0] == 0 && gy521Values[0] == 0) return;

            // Convert LSB values to rad/s or m/s^2 using the conversion factors
            double gyroX = gy521Values[0] * gy521.gyroConversionFactor;
            double gyroY = gy521Values[1] * gy521.gyroConversionFactor;
            double gyroZ = gy521Values[2] * gy521.gyroConversionFactor;

            double accelX = gy521Values[3] * gy521.accelConversionFactor;
            double accelY = gy521Values[4] * gy521.accelConversionFactor;
            double accelZ = gy521Values[5] * gy521.accelConversionFactor;

            rclcpp::Time currentTime = this->get_clock()->now();

            // Calculate the time since the last PID cycle
            double dt = (currentTime - previousTime).seconds();
            previousTime = currentTime;

            // Get the roll and pitch values based on the accelerometer data
            double accelRoll = std::atan2(
                accelY, 
                std::sqrt(std::pow(accelZ, 2) + std::pow(accelX, 2))
            );
            double accelPitch = std::atan2(
                accelX, 
                std::sqrt(std::pow(accelZ, 2) + std::pow(accelY, 2))
            );

            // Get the change in roll and pitch from the gyroscope
            double gyroRoll = gyroX * dt;
            double gyroPitch = gyroY * dt;

            currentRollVelocity = gyroX;
            currentPitchVelocity = gyroY;

            // Get the roll and pitch using a complementary filter
            currentRoll = gyroAccelRatio * (currentRoll + gyroRoll) + (1 - gyroAccelRatio) * accelRoll;
            currentPitch = gyroAccelRatio * (currentPitch + gyroPitch) + (1 - gyroAccelRatio) * accelPitch;

            // This PID script runs assuming that the target pitch and roll is 0
            double proportionalRoll = currentRoll * proportional_constant; 
            double derivativeRoll = currentRollVelocity * derivative_constant; 
            
            double proportionalPitch = currentPitch * proportional_constant; 
            double derivativePitch = currentPitchVelocity * derivative_constant; 

            // Clamp the offset values to prevent the leg from moving to an invalid location
            correctionRoll = std::clamp(correctionRoll + proportionalRoll + derivativeRoll, -15.0, 15.0);
            correctionPitch = std::clamp(correctionPitch + proportionalPitch + derivativePitch, -15.0, 15.0);

            interfaces::msg::BalanceCorrection msg;

            msg.roll_correction = correctionRoll;
            msg.pitch_correction = correctionPitch;

            // Send the updated values to the control node
            _balanceCorrectionPublisher->publish(msg);
        }

        void i2c_write(void* buf, uint8_t size) {
            /* ------
                Purpose:
                    Generic i2c register write function
                Parameters:
                    void* buf - Values being written to the registers. The first value is the starting register and the 
                                remaining values are written to subsequent registers 
                    uint8_t size - The number of bits that should be written to the device
                Return 
                    None
            ------ */ 

            if (write(fd_, buf, size) != size) throw std::runtime_error("I2C write failed");
        }

        void start_gy521(){
            /* ------
                Purpose:
                    This function is used to set all of the gy521 registers on startup so the device functions properly
                Parameters:
                    None
                Return 
                    None
            ------ */ 

            uint8_t start_buf[2] = {POWER_MANAGEMENT, 0x00};
            uint8_t reset_buf[2] = {POWER_MANAGEMENT, 0x80};

            // Reset all of the device register values
            i2c_write(&reset_buf, 2);

            // Wait for the changes to be applied. It is assumed 4 seconds is enough although the value can 
            // probably be much lower
            rclcpp::sleep_for(std::chrono::seconds(4));

            // Start the gy521
            i2c_write(&start_buf, 2);
        }

        void run_calibration_cycle(){
            
            /* ------
                Purpose:
                    This function is called each calibration cycle to update the totals and set the offsets once enough
                    samples have been taken
                Parameters:
                    None
                Return 
                    None
            ------ */ 

            // Read the gyroscope and accelerometer data from the gy521
            std::array<int16_t, 6> gy521Data = i2c_read_values();

            // Add the values to the totals
            gy521.gyroXTotal += gy521Data[0];
            gy521.gyroYTotal += gy521Data[1];
            gy521.gyroZTotal += gy521Data[2];

            gy521.accelXTotal += gy521Data[3];
            gy521.accelYTotal += gy521Data[4];
            gy521.accelZTotal += gy521Data[5];

            gy521.calibrationSamples += 1;

            // Once the desired number of calibration cycles has been reached the offset values should be applied
            if(gy521.calibrationSamples == gy521.targetCalibrationSamples)
            {
                isCalibrating = false;

                // Acceleration offset registers have a value by default so that needs to be included in the offset.
                int16_t offsetAccelXCur = i2c_read_register_16(0x06);
                int16_t offsetAccelYCur = i2c_read_register_16(0x08);
                int16_t offsetAccelZCur = i2c_read_register_16(0x0A);

                // Calculate the offset values for each register using the mean value
                // Note: there are different modes for the GY521 accelerometer (+-2g +-4g +-8g, +-16g) and gyroscope. For this project we are
                // using the minimum full range for both the accelerometer and gyroscope. However, the offset values are for the largest full range (+-16g for accelerometer)
                // this means that the offset values need to be converted using an offset factor
                int16_t offsetGyroX = std::floor((gy521.gyroXTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 
                int16_t offsetGyroY = std::floor((gy521.gyroYTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 
                int16_t offsetGyroZ = std::floor((gy521.gyroZTotal / gy521.targetCalibrationSamples) * gy521.gyroOffsetFactor) * -1; 

                // The accelerometer offsets are similar to those above, however, they have predefined values in their registers, the first bit is used for thermal 
                // calibration and cannot be changed, and z accel should have a default value of -1g
                int16_t offsetAccelX = std::floor((gy521.accelXTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) * -1 + offsetAccelXCur; 
                int16_t offsetAccelY = std::floor((gy521.accelYTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) * -1 + offsetAccelYCur; 
                int16_t offsetAccelZ = std::floor((-16384 - gy521.accelZTotal / gy521.targetCalibrationSamples) * gy521.accelOffsetFactor) + offsetAccelZCur; 

                // Bit 0 of the acceleration registers is used for temperature control and should not be changed.
                offsetAccelX = (offsetAccelX & ~1) | (1 & offsetAccelXCur);
                offsetAccelY = (offsetAccelY & ~1) | (1 & offsetAccelYCur);
                offsetAccelZ = (offsetAccelZ & ~1) | (1 & offsetAccelZCur);

                // Buffer arrays are created with the starting register and the corresponding values that should be written
                // Note: since the ADC is 16bit and the registers are 8bit they are utilizing high and low registers which
                //       is why the register values are split  
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

                // Write the accelerometer and gyroscrope offsets
                i2c_write(&accelOffsets, 7);

                i2c_write(&gyroOffsets, 7);
            }
        }

        std::array<int16_t, 6> i2c_read_values() {
            /* ------
                Purpose:
                    Read the accelerometer and gyroscope values from the GY521 over I2C 
                Parameters:
                    None
                Return 
                    std::array<int16_t, 6> - Returns the IMU data in the following order: {gyx, gyy, gyz, accx, accy, accz}
            ------ */ 

            std::array<uint8_t, 12> registerData;

            // Get the gyro and accel start addresses
            // ToDo: Since these are just const variables they don't have to be re-declared here since we can
            //       just pass the address below
            uint8_t gyroStartAddr = GYRO_START;
            uint8_t accelStartAddr = ACCEL_START;

            std::array<int16_t, 6> returnData = {0, 0, 0, 0, 0, 0};

            // Get the gyro and accel values from the IMU. If the size is wrong then just return an array of zeros by default
            if (write(fd_, &gyroStartAddr, 1) != 1) return returnData; 
            if (read(fd_, &registerData, 6) != 6) return returnData; 

            if (write(fd_, &accelStartAddr, 1) != 1) return returnData; 
            if (read(fd_, &registerData[0] + 6, 6) != 6) return returnData; 

            // Loop over each 8 bit interger value and form the full 16bit value
            for(uint16_t index = 0; index < registerData.size(); index += 2)
            {
                returnData[index/2] = (registerData[index] << 8 | registerData[index + 1]);
            }

            return returnData;
        }

        int16_t i2c_read_register_16(uint8_t addr){
            /* ------
                Purpose:
                    This is for reading values from the offset registers. This is used in calibration since the offset registers
                    are 8bits but the values are 16bits
                Parameters:
                    uint8_t addr - The address of the first register you need to read
                Return 
                    int16_t - Returns the full 16bit value from the two registers
                ------ */ 
                
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