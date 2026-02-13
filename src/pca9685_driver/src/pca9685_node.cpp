#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>

class PCA9685Driver : public rclcpp::Node {
public:
    PCA9685Driver() : Node("pca9685_driver") {
        // Declare the parameters for the pca9685
        i2c_dev_ = this->declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
        i2c_addr_ = this->declare_parameter<int>("i2c_addr", 0x40);
        pwm_hz_ = this->declare_parameter<double>("pwm_hz", 50.0);
        update_hz_ = this->declare_parameter<double>("update_hz", 50.0);

        // This is the max and min pulse width, 
        min_us_ = this->declare_parameter<double>("min_pulse_us", 544.0);
        max_us_ = this->declare_parameter<double>("max_pulse_us", 2400.0);
 
        // These channels should be passed in as parameters rather than hardcoded. Look into this
        // FL1 - Front, Left, 1st Motor
        // 1st Motor - Shoulder Joint
        // 2nd Motor - Upper Joint
        // 3rd Motor - Lower Elbow Joint

        servo_map_["servoFL1"] = 0;
        servo_map_["servoFL2"] = 1;
        servo_map_["servoFL3"] = 2;

        servo_map_["servoFR1"] = 4;
        servo_map_["servoFR2"] = 5;
        servo_map_["servoFR3"] = 6;

        servo_map_["servoBL1"] = 8;
        servo_map_["servoBL2"] = 9;
        servo_map_["servoBL3"] = 10;

        servo_map_["servoBR1"] = 12;
        servo_map_["servoBR2"] = 13;
        servo_map_["servoBR3"] = 14;

        open_i2c();
        init_pca9685();

        // This creates a subscription for "servo/command" topic. rclcpp:QoS(10) declares how many messages will be stored in the queue
        // [this](param) {} is a lamba function. Cool. Which gets the SharedPtr for the JointState (SharedPtr is the standard value passed to lambda functions)
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "servo/command", rclcpp::QoS(10), 
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {on_command(*msg);});

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_hz_),
            [this]() { write_outputs(); }
        );

    }

    ~PCA9685Driver() override {
        if (fd_ >= 0) close(fd_);
    }

    private:
        // PCA9685 Registers
        static constexpr uint8_t MODE1 = 0x00;
        static constexpr uint8_t MODE2 = 0x01;
        static constexpr uint8_t PRESCALE = 0xFE;
        static constexpr uint8_t LED0_ON_L = 0x06;

        // MODE1 bits
        static constexpr uint8_t MODE1_SLEEP = 0x10;
        static constexpr uint8_t MODE1_AI = 0x20;
        static constexpr uint8_t MODE1_RESTART = 0x80;
        
        // MODE2 bits
        static constexpr uint8_t MODE2_OUTDRV = 0x04;

        void open_i2c() {
            // Open the linux i2c bus 
            fd_ = open(i2c_dev_.c_str(), O_RDWR);
            std::cout << i2c_dev_.c_str() << std::endl;
            // If the integer is < 0 then the i2c bus has already been opened
            if  (fd_ < 0) throw std::runtime_error("Failed to open I2C device"); 

            // If the slave address cannot be sent then throw an error
            if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) throw std::runtime_error("Failed to set I2C slave address");
        }

        void i2c_write_reg(uint8_t reg, uint8_t value) {
            uint8_t buf[2] = {reg, value};
            // Checks to make sure that two bytes have been written to the i2c bus
            if (write(fd_, buf, 2) != 2) throw std::runtime_error("I2C write failed");
        }

        uint8_t i2c_read_reg(uint8_t reg) {
            // I2C need to perform a write to a register to select it before reading its vale
            if (write(fd_, &reg, 1) != 1) throw std::runtime_error("I2C reg select failed");
            uint8_t val = 0;
            if (read(fd_, &val, 1) != 1) throw std::runtime_error("I2C read failed");
            return val;
        }

        void init_pca9685() {
            // Set OUTDRV
            i2c_write_reg(MODE2, MODE2_OUTDRV);
            
            // Sleep to set prescale
            uint8_t mode1 = i2c_read_reg(MODE1);
            // Forces the device to restart and puts the oscillator into a low power mode
            i2c_write_reg(MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP);

            set_pwm_freq(pwm_hz_);

            // Wake + auto increment
            i2c_write_reg(MODE1, MODE1_AI);
            // Restart bit can be set after oscillator is running; simple approach:
            usleep(1000);
            i2c_write_reg(MODE1, MODE1_AI | MODE1_RESTART);
        }

        void set_pwm_freq(double hz) {
            // PCA9685 internal osc is typically 25 MHz
            constexpr double osc_hz = 25000000.0;
            
            double prescale_f = (osc_hz / (4096.0 * hz)) - 1.0;
            uint8_t prescale = static_cast<uint8_t>(std::lround(prescale_f));

            i2c_write_reg(PRESCALE, prescale);
        }

        void on_command(const sensor_msgs::msg::JointState &msg) {
            // message contains an array of servo names and the corresponding positions

            // store latest desired positions (radians)
            for(size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
                target_pos_[msg.name[i]] = msg.position[i];
            }
        }

        void set_channel_ticks(std::array<uint16_t> on, std::array<uint16_t> off) {
            // on is the number of ticks after the pulse starts that it will go to high
            // off is the number of ticks after the pulse starts that it will return back to low
            // on is normally 1 for servos
            // since the registers are only 8 bits we need to split them into two seperate values into a lower and higher value in the buffer

            // Will need to rewrite this to do batch updates
            // Need to set PCA9685 to auto increment;

            //uint8_t reg = LED0_ON_L; //+ 4 * channel;

            // Test this to make sure that the write for servos works. Need to account for empty channels (3, 7, 11, 15)
            std::array<uint8_t, 49> buff = {};
            buff[0] = LED0_ON_L;

            uint8_t num_motors = sizeOf(on) / sizeOf(uint16_t);;

            for(uint8_t index = 0; index < num_motors; index ++){
                buff[index * 4 + 1] = static_cast<uint8_t>(on[index] & 0xFF);
                buff[index * 4 + 2] = static_cast<uint8_t>((on[index] >> 8) & 0xFF),
                buff[index * 4 + 3] = static_cast<uint8_t>(off[index] & 0xFF),
                buff[index * 4 + 4] = static_cast<uint8_t>((off[index] >> 8) & 0xFF)
            }

            // uint8_t buff[5] = {
            //     reg,
            //     static_cast<uint8_t>(on & 0xFF),
            //     static_cast<uint8_t>((on >> 8) & 0xFF),
            //     static_cast<uint8_t>(off & 0xFF),
            //     static_cast<uint8_t>((off >> 8) & 0xFF)
            // };
            if (write(fd_, buff, 49) != 5) throw std::runtime_error("I2C channel write failed");
        }

        double rad_to_us(double rad) const {
            // Example: map [-pi/2, pi/2] to [min_us_, max_us_]

            double t = (rad + M_PI_2) / (M_PI);
            if(t < 0.0) t = 0.0;
            if(t > 1.0) t = 1.0;
            return min_us_ + t * (max_us_ - min_us_);
        }

        uint16_t us_to_ticks(double pulse_us) const {
            double period_us = 1e6 / pwm_hz_;
            double ticks = (pulse_us / period_us) * 4096.0;
            if(ticks < 0.0) ticks = 0.0;
            if(ticks > 4095.0) ticks = 4095.0;
            return static_cast<uint16_t>(std::lround(ticks));
        }

        void write_outputs() {
            std::array<uint16_t, 12> off_ticks = {}
            std::array<uint16_t, 12> on_ticks = {0};

            // Index was added in. This might break
            for (const auto& [index, name, ch] : servo_map_) {
                auto it = target_pos_.find(name);
                if(it == target_pos_.end()) continue;
                
                double pulse_us = rad_to_us(it->second);
                uint16_t off = us_to_ticks(pulse_us);   

                off_ticks[index] = off;
            }
            
            set_channel_ticks(ch, on_ticks, off_ticks); // Removed ch (channel) from set_channel_ticks
        }

        std::string i2c_dev_;
        int i2c_addr_;
        double pwm_hz_;
        double update_hz_;
        double max_us_;
        double min_us_;

        int fd_{-1};

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unordered_map<std::string, uint8_t> servo_map_;
        std::unordered_map<std::string, double> target_pos_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685Driver>());
    rclcpp::shutdown();

    return 0;
}