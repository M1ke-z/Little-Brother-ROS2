#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <unordered_map>
#include <vector>
#include <span>
#include <cmath>
#include <cstdint>
#include <string>

#include "interfaces/msg/motor_position.hpp"

class PCA9685Driver : public rclcpp::Node {
public:
    PCA9685Driver() : Node("pca9685_driver") {
        // Declare the parameters for the pca9685
        i2cDev = this->declare_parameter<std::string>("i2c_dev", "/dev/i2c-1");
        i2cAddr = this->declare_parameter<int>("i2c_addr", 0x40);
        pwmHz = this->declare_parameter<double>("pwm_hz", 50.0);
        updateHz = this->declare_parameter<double>("update_hz", 50.0);

        // Open the I2C connection and initialize the PCA9685
        open_i2c();
        init_pca9685();

        // Declare all subscriptions, publishers, and timers

        // This recieves pulse durations from the control node for each motor
        sub_ = this->create_subscription<interfaces::msg::MotorPosition>(
            "servo/command", rclcpp::QoS(10), 
            [this](interfaces::msg::MotorPosition::SharedPtr msg) {on_command(*msg);});

        // This timer will send updated values to the PCA9685 at the desired frequency
        // ToDo: Although the PCA9685 will only update at 50Hz values can be sent much faster. There might be a way
        //       to queue future foot positions which may help reduce motor jitter without switching to a microcontroller 
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / updateHz),
            [this]() { write_outputs(); }
        );

    }

    // PCA9685 Deconstructor
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

        std::string i2cDev; // I2C string
        
        int i2cAddr; //  Address of the PCA9685
        double pwmHz; // PWM update frequency ToDo: Unlike the MG996R servos the DS3230 PRO can update at frequencies 
                      // greater than 50Hz which could make motions smoother
        double updateHz; // How often updated values will be sent to the PCA9685

        // Declare the file descriptor for the I2C Bus
        int fd_{-1};

        // Deca=lare the subscription, publisher, and timer variables
        rclcpp::Subscription<interfaces::msg::MotorPosition>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unordered_map<uint16_t, double> targetPos;

        void open_i2c() {
            /* ------
                Purpose:
                    Opens the I2C bus for the PCA9685
                Parameters:
                    None
                Return 
                    None
            ------ */ 

            // Open the linux i2c bus 
            fd_ = open(i2cDev.c_str(), O_RDWR);

            // If the integer is < 0 then the i2c bus has already been opened
            if (fd_ < 0) throw std::runtime_error("Failed to open I2C device"); 

            // If the slave address cannot be sent then throw an error
            if (ioctl(fd_, I2C_SLAVE, i2cAddr) < 0) throw std::runtime_error("Failed to set I2C slave address");

            return;
        }

        void i2c_write_reg(uint8_t reg, uint8_t value) {
            /* ------
                Purpose:
                    Generic i2c register write function
                Parameters:
                    uint8_t reg - Register that the value should be written to
                    uint8_t value - Value that should be written to the register
                Return 
                    None
                ------ */ 

            uint8_t buf[2] = {reg, value};
            // Checks to make sure that two bytes have been written to the i2c bus
            if (write(fd_, buf, 2) != 2) throw std::runtime_error("I2C write failed");

            return;
        }

        uint8_t i2c_read_reg(uint8_t reg) {
            /* ------
                Purpose:
                    Generic i2c register read function
                Parameters:
                    uint8_t reg - Register that the value should be read from
                Return 
                    uint8_t - Returns the value of the register
                ------ */ 

            // Write to the PCA9685 to set the register pointer
            if (write(fd_, &reg, 1) != 1) throw std::runtime_error("I2C reg select failed");
            
            uint8_t val = 0;

            // Read the register value
            if (read(fd_, &val, 1) != 1) throw std::runtime_error("I2C read failed");
            return val;
        }

        void init_pca9685() {
            /* ------
                Purpose:
                    Initialize the PCA9685 and set register values
                Parameters:
                    None
                Return 
                    None
                ------ */ 

            // Set OUTDRV
            i2c_write_reg(MODE2, MODE2_OUTDRV);
            
            // Sleep to set prescale
            uint8_t mode1 = i2c_read_reg(MODE1);

            // Forces the device to restart and puts the oscillator into a low power mode
            i2c_write_reg(MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP);

            set_pwm_freq(pwmHz);

            // Wake + auto increment
            i2c_write_reg(MODE1, MODE1_AI);

            // Restart bit can be set after oscillator is running
            usleep(1000);
            i2c_write_reg(MODE1, MODE1_AI | MODE1_RESTART);

            return;
        }

        void set_pwm_freq(double hz) {
            /* ------
                Purpose:
                    Set the PCA9685 internal PWM frequency
                Parameters:
                    double hz - Target frequency
                Return 
                    None
               ------ */ 

            // PCA9685 internal osc is typically 25 MHz
            constexpr double osc_hz = 25000000.0;
            
            // Scale the target frequency and convert it to an 8bit integer value
            double prescaleFreq = (osc_hz / (4096.0 * hz)) - 1.0;
            uint8_t prescale = static_cast<uint8_t>(std::lround(prescaleFreq));

            i2c_write_reg(PRESCALE, prescale);

            return;
        }

        void on_command(const interfaces::msg::MotorPosition &msg) {
            /* ------
                Purpose:
                    This function recieves the updated motor pulse durations from the control node and updates the PCA9685
                Parameters:
                    const interfaces::msg::MotorPosition &msg - Motor pulse durations
                Return 
                    None
               ------ */ 

            // message contains an array of servo names and the corresponding positions

            // Store the updated motor positions as a number of ticks
            for(size_t i = 0; i < msg.motor.size() && i < msg.pulses.size(); i++) {
                targetPos[msg.motor[i]] = us_to_ticks(msg.pulses[i]);
            }

            return;
        }

        void set_channel_ticks(std::array<uint16_t, 16> on, std::array<uint16_t, 16> off) {
            /* ------
                Purpose:
                    This function takes the position for each of the motors in ticks and sends those updated values to the 
                    PCA9685
                Parameters:
                    std::array<uint16_t, 16> on - The number of ticks after the pulse starts that it will go to high (Normally 1 for servos)
                    std::array<uint16_t, 16> off - The number of ticks after the pulse starts that it will return back to low
                Return 
                    None
               ------ */ 

            // since the registers are only 8 bits we need to split them into two seperate values into a lower and higher value in the buffer

            std::array<uint8_t, 65> buff = {};
            buff[0] = LED0_ON_L;

            uint8_t numChannels = on.size();

            // Split each tick value into high and low values and write them to the buffer
            for(uint8_t index = 0; index < numChannels; index++){
                buff[index * 4 + 1] = static_cast<uint8_t>(on[index] & 0xFF);
                buff[index * 4 + 2] = static_cast<uint8_t>((on[index] >> 8) & 0xFF);
                buff[index * 4 + 3] = static_cast<uint8_t>(off[index] & 0xFF);
                buff[index * 4 + 4] = static_cast<uint8_t>((off[index] >> 8) & 0xFF);
            }

            // Write the new values to the PCA9685
            if (write(fd_, &buff, 65) != 65) throw std::runtime_error("I2C channel write failed");

            return;
        }

        void write_outputs() {
            /* ------
                Purpose:
                    This function gets all of the target positions, formats them in an array, and calls a function to update the PCA9685
                Parameters:
                    None
                Return 
                    None
               ------ */ 

            std::array<uint16_t, 16> offTicks = {};
            std::array<uint16_t, 16> onTicks = {};

            // Loop over each value in targetPos and update the offTicks value
            for (const auto& [servo_num, servoTicks] : targetPos) {
                offTicks[servo_num] = servoTicks;
            }

            // onTicks by default is all zero since we don't need to adjust when the start time of the PWM signal
            set_channel_ticks(onTicks, offTicks); 

            return;
        }

        uint16_t us_to_ticks(double pulseUs) const {
            /* ------
                Purpose:
                    Takes a pulse duration in microseconds and converts it to a ticks value
                Parameters:
                    double pulseUs - Pulse duration in microseconds
                Return 
                    uint16_t - The number of ticks that the pwm signal should be high for
               ------ */ 

            double periodUs = 1e6 / pwmHz; // Calculate the total pulse period
            // Convert the pulse duration to a tick quantity
            // Note: PCA9685 has a 12 bit internal counter for PWM signals (2^12=4096)
            double ticks = (pulseUs / periodUs) * 4096.0; 

            // Check to make sure that ticks is within a valid range
            if(ticks < 0.0) ticks = 0.0;
            if(ticks > 4095.0) ticks = 4095.0;

            // Return the pulse length in ticks as an integer
            return static_cast<uint16_t>(std::lround(ticks));
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685Driver>());
    rclcpp::shutdown();

    return 0;
}