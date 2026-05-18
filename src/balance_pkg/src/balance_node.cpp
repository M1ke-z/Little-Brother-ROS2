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
    int64_t gyro_x_total = 0;
    int64_t gyro_y_total = 0;
    int64_t gyro_z_total = 0;

    int64_t accel_x_total = 0;
    int64_t accel_y_total = 0;
    int64_t accel_z_total = 0;

    // The offset registers use a different scale than the readout so it needs to be converted
    const double gyro_offset_factor =  0.25; 
    const double accel_offset_factor = 0.125; 

    // The offset used to convert LSB to phyiscal values
    const double accel_conversion_factor = 4.0 / std::pow(2, 16); // g/lsb
    const double gyro_conversion_factor = (500.0 / std::pow(2, 16)) * (M_PI/180.0); // rad/(lsb*sec)

    const uint16_t target_calibration_samples = 1000;

    int16_t calibration_samples = 0;
};

class balance : public rclcpp::Node {
public:
    balance() : Node("balance_node") {
        i2c_dev_ = this->declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
        i2c_addr_ = this->declare_parameter<int>("i2c_addr", 0x68);

        open_i2c();

        start_gy521();

        is_calibrating = true;

        previous_time = this->get_clock()->now();

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
                this->correction_roll = 0.0;
                this->correction_pitch = 0.0;
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 400.0),
            [this]() {
                if(is_calibrating) run_calibration_cycle();
                else runPID(); 
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
        
        rclcpp::Time previous_time;

        double current_roll;
        double current_pitch;

        double current_roll_velocity;
        double current_pitch_velocity;

        double correction_roll;
        double correction_pitch;

        const double gyro_accel_ratio = 0.95;

        double proportional_constant = 0.8;
        double derivative_constant = 0.15;
        const double integral_constant = 0;

        std::string i2c_dev_;
        int i2c_addr_;

        int fd_{-1};

        bool is_calibrating = false;

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

        void handle_imu_update(const sensor_msgs::msg::Imu& imu_data) {
            // RCLCPP_INFO(this->get_logger(), "Angular Veclocity - X: %.2f Y: %.2f Z: %.2f", imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
            // RCLCPP_INFO(this->get_logger(), "Linear Acceleration - X: %.2f Y: %.2f Z: %.2f", imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);

            rclcpp::Time current_time = this->get_clock()->now();

            double dt = (current_time - previous_time).seconds();
            previous_time = current_time;

            double accel_roll = std::atan2(
                imu_data.linear_acceleration.y, 
                std::sqrt(std::pow(imu_data.linear_acceleration.z, 2) + std::pow(imu_data.linear_acceleration.x, 2))
            );
            double accel_pitch = std::atan2(
                imu_data.linear_acceleration.x, 
                std::sqrt(std::pow(imu_data.linear_acceleration.z, 2) + std::pow(imu_data.linear_acceleration.y, 2))
            );

            double gyro_roll = imu_data.angular_velocity.x * dt;
            double gyro_pitch = imu_data.angular_velocity.y * dt;

            current_roll_velocity = imu_data.angular_velocity.x;
            current_pitch_velocity = imu_data.angular_velocity.y;

            current_roll = gyro_accel_ratio * (current_roll + gyro_roll) + (1 - gyro_accel_ratio) * accel_roll;
            current_pitch = gyro_accel_ratio * (current_pitch + gyro_pitch) + (1 - gyro_accel_ratio) * accel_pitch;

            //RCLCPP_INFO(this->get_logger(), "Roll: %.6f, %.6f, Pitch: %.6f, %.2f", gyro_roll, accel_roll, gyro_pitch, accel_pitch);
            //RCLCPP_INFO(this->get_logger(), "Roll: %.6f, Pitch: %.6f", current_roll, current_pitch);
        }

        void runPID() {

            std::array<int16_t, 6> gy521_values;

            gy521_values = i2c_read_values();

            // If the first three values are zero assume that the read failed.
            if (gy521_values[0] == 0 && gy521_values[0] == 0 && gy521_values[0] == 0) return;

            // RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d", 
            //     gy521_values[0], gy521_values[1], gy521_values[2], gy521_values[3], gy521_values[4], gy521_values[5]
            // );

            double gyro_x = gy521_values[0] * gy521.gyro_conversion_factor;
            double gyro_y = gy521_values[1] * gy521.gyro_conversion_factor;
            double gyro_z = gy521_values[2] * gy521.gyro_conversion_factor;

            double accel_x = gy521_values[3] * gy521.accel_conversion_factor;
            double accel_y = gy521_values[4] * gy521.accel_conversion_factor;
            double accel_z = gy521_values[5] * gy521.accel_conversion_factor;

            rclcpp::Time current_time = this->get_clock()->now();

            double dt = (current_time - previous_time).seconds();
            previous_time = current_time;

            double accel_roll = std::atan2(
                accel_y, 
                std::sqrt(std::pow(accel_z, 2) + std::pow(accel_x, 2))
            );
            double accel_pitch = std::atan2(
                accel_x, 
                std::sqrt(std::pow(accel_z, 2) + std::pow(accel_y, 2))
            );

            double gyro_roll = gyro_x * dt;
            double gyro_pitch = gyro_y * dt;

            current_roll_velocity = gyro_x;
            current_pitch_velocity = gyro_y;

            current_roll = gyro_accel_ratio * (current_roll + gyro_roll) + (1 - gyro_accel_ratio) * accel_roll;
            current_pitch = gyro_accel_ratio * (current_pitch + gyro_pitch) + (1 - gyro_accel_ratio) * accel_pitch;

            // RCLCPP_INFO(this->get_logger(), "Current Roll: %.6f, Current Pitch: %.6f", 
            //     current_roll, current_pitch
            // );

            // This PID script runs assuming that the target pitch and roll is 0
            double proportional_roll = current_roll * proportional_constant; 
            double derivative_roll = current_roll_velocity * derivative_constant; 
            
            double proportional_pitch = current_pitch * proportional_constant; 
            double derivative_pitch = current_pitch_velocity * derivative_constant; 

            correction_roll = std::clamp(correction_roll + proportional_roll + derivative_roll, -15.0, 15.0);
            correction_pitch = std::clamp(correction_pitch + proportional_pitch + derivative_pitch, -15.0, 15.0);

            interfaces::msg::BalanceCorrection msg;

            msg.roll_correction = correction_roll;
            msg.pitch_correction = correction_pitch;

            _balanceCorrectionPublisher->publish(msg);
        }

        void i2c_write(void* buf, uint8_t size) {
            if (write(fd_, buf, size) != size) throw std::runtime_error("I2C write failed");
        }

        void start_gy521(){
            uint8_t start_buf[2] = {POWER_MANAGEMENT, 0x00};
            uint8_t reset_buf[2] = {POWER_MANAGEMENT, 0x80};
            //uint8_t low_pass_buf[2] = {CONFIG, 0x03};

            i2c_write(&reset_buf, 2);

            rclcpp::sleep_for(std::chrono::seconds(4));

            i2c_write(&start_buf, 2);

            //i2c_write(&low_pass_buf, 2);
        }

        void run_calibration_cycle(){
            
            std::array<int16_t, 6> gy521_data = i2c_read_values();

            gy521.gyro_x_total += gy521_data[0];
            gy521.gyro_y_total += gy521_data[1];
            gy521.gyro_z_total += gy521_data[2];

            gy521.accel_x_total += gy521_data[3];
            gy521.accel_y_total += gy521_data[4];
            gy521.accel_z_total += gy521_data[5];

            gy521.calibration_samples += 1;

            if(gy521.calibration_samples == gy521.target_calibration_samples)
            {
                is_calibrating = false;

                // Acceleration offset registers have a value by default so that needs to be included in the offset.
                int16_t offset_accel_x_cur = i2c_read_register_16(0x06);
                int16_t offset_accel_y_cur = i2c_read_register_16(0x08);
                int16_t offset_accel_z_cur = i2c_read_register_16(0x0A);

                int16_t offset_gyro_x = std::floor((gy521.gyro_x_total / gy521.target_calibration_samples) * gy521.gyro_offset_factor) * -1; 
                int16_t offset_gyro_y = std::floor((gy521.gyro_y_total / gy521.target_calibration_samples) * gy521.gyro_offset_factor) * -1; 
                int16_t offset_gyro_z = std::floor((gy521.gyro_z_total / gy521.target_calibration_samples) * gy521.gyro_offset_factor) * -1; 

                int16_t offset_accel_x = std::floor((gy521.accel_x_total / gy521.target_calibration_samples) * gy521.accel_offset_factor) * -1 + offset_accel_x_cur; 
                int16_t offset_accel_y = std::floor((gy521.accel_y_total / gy521.target_calibration_samples) * gy521.accel_offset_factor) * -1 + offset_accel_y_cur; 
                int16_t offset_accel_z = std::floor((-16384 - gy521.accel_z_total / gy521.target_calibration_samples) * gy521.accel_offset_factor) + offset_accel_z_cur; 

                // Bit 0 of the acceleration registers is used for temperature control and should not be changed.
                offset_accel_x = (offset_accel_x & ~1) | (1 & offset_accel_x_cur);
                offset_accel_y = (offset_accel_y & ~1) | (1 & offset_accel_y_cur);
                offset_accel_z = (offset_accel_z & ~1) | (1 & offset_accel_z_cur);

                std::array<uint8_t, 7> accel_offsets = {
                    0x06, 
                    static_cast<uint8_t>(offset_accel_x >> 8), static_cast<uint8_t>(offset_accel_x & 0x00FF), 
                    static_cast<uint8_t>(offset_accel_y >> 8), static_cast<uint8_t>(offset_accel_y & 0x00FF),
                    static_cast<uint8_t>(offset_accel_z >> 8), static_cast<uint8_t>(offset_accel_z & 0x00FF)
                };

                std::array<uint8_t, 7> gyro_offsets = {
                    0x13, 
                    static_cast<uint8_t>(offset_gyro_x >> 8), static_cast<uint8_t>(offset_gyro_x & 0x00FF), 
                    static_cast<uint8_t>(offset_gyro_y >> 8), static_cast<uint8_t>(offset_gyro_y & 0x00FF),
                    static_cast<uint8_t>(offset_gyro_z >> 8), static_cast<uint8_t>(offset_gyro_z & 0x00FF)
                };

                i2c_write(&accel_offsets, 7);

                i2c_write(&gyro_offsets, 7);
            }
        }

        std::array<int16_t, 6> i2c_read_values() {
            std::array<uint8_t, 12> register_data;

            uint8_t gyro_start_addr = GYRO_START;
            uint8_t accel_start_addr = ACCEL_START;

            std::array<int16_t, 6> return_data = {0, 0, 0, 0, 0, 0};


            if (write(fd_, &gyro_start_addr, 1) != 1) return return_data; 
            if (read(fd_, &register_data, 6) != 6) return return_data; 

            if (write(fd_, &accel_start_addr, 1) != 1) return return_data; 
            if (read(fd_, &register_data[0] + 6, 6) != 6) return return_data; 

            

            for(uint16_t index = 0; index < register_data.size(); index += 2)
            {
                return_data[index/2] = (register_data[index] << 8 | register_data[index + 1]);
            }

            return return_data;
        }

        int16_t i2c_read_register_16(uint8_t addr){
            std::array<uint8_t, 2> register_data;

            if(write(fd_, &addr, 1) != 1) throw std::runtime_error("I2C reg select failed"); 
            if(read(fd_, &register_data, 2) != 2) throw std::runtime_error("I2C read failed");

            return register_data[0] << 8 | register_data[1];
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<balance>());
    rclcpp::shutdown();

    return 0;
}