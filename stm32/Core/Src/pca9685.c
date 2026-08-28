#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>
#include "pca9685.h"


// PCA9686 General Variables
#define I2C_ADDR (0x40 << 1)
#define PWM_HZ 100.0
#define UPDATE_HZ 50.0
#define OSC_HZ 25000000.0

// PCA9685 Registers
#define MODE1 0x00
#define MODE2 0x01
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

#define MODE1_SLEEP 0x10
#define MODE1_AI 0x20
#define MODE1_RESTART 0x80

#define MODE2_OUTDRV 0x04

I2C_HandleTypeDef *hi2c1_ref1;

void pca9685_setup(I2C_HandleTypeDef *hi2c1_ref) {
    hi2c1_ref1 = hi2c1_ref;

    uint8_t outdrv_buffer[2] = {MODE2, MODE2_OUTDRV}; 
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, outdrv_buffer, 2, HAL_MAX_DELAY);

    uint32_t error = HAL_I2C_GetError(hi2c1_ref1);

    uint8_t mode1_select = MODE1;
    uint8_t mode1 = 0;
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, &mode1_select, 1, HAL_MAX_DELAY);

    uint32_t error1 = HAL_I2C_GetError(hi2c1_ref1);

    HAL_I2C_Master_Receive(hi2c1_ref1, I2C_ADDR, &mode1, 1, HAL_MAX_DELAY);

    uint8_t restart_buffer[2] = {MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP};
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, restart_buffer, 2, HAL_MAX_DELAY);

    double prescale_freq = (OSC_HZ / (4096.0 * PWM_HZ)) - 1;
    uint8_t prescale_buffer[2] = {PRESCALE, (uint8_t)lround(prescale_freq)};
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, prescale_buffer, 2, HAL_MAX_DELAY);

    uint8_t wake_buffer[2] = {MODE1, MODE1_AI};
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, wake_buffer, 2, HAL_MAX_DELAY);

    HAL_Delay(1000);

    uint8_t run_buffer[2] = {MODE1, MODE1_AI | MODE1_RESTART};
    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, run_buffer, 2, HAL_MAX_DELAY);

    return;
}

uint16_t us_to_ticks(double pulseUs){ 
    double periodUs = 1e6 / PWM_HZ;

    double ticks = (pulseUs / periodUs) * 4096.0;

    if(ticks < 0.0) ticks = 0.0;
    else if(ticks > 4095.0) ticks = 4095.0;

    return (uint16_t)lround(ticks);
}

void set_channel_ticks(uint16_t off[16], uint16_t on[16]) {
    uint8_t buff[65] = {};
    buff[0] = LED0_ON_L;

    uint8_t numChannels = 16;

    for(uint8_t index = 0; index < numChannels; index++) {
        buff[index * 4 + 1] = (uint8_t)(us_to_ticks(on[index]) & 0xFF);
        buff[index * 4 + 2] = (uint8_t)(us_to_ticks(on[index]) >> 8 & 0xFF);
        buff[index * 4 + 3] = (uint8_t)(us_to_ticks(off[index]) & 0xFF);
        buff[index * 4 + 4] = (uint8_t)(us_to_ticks(off[index]) >> 8 & 0xFF);
    }

    HAL_I2C_Master_Transmit(hi2c1_ref1, I2C_ADDR, buff, 65, HAL_MAX_DELAY);

    uint32_t error = HAL_I2C_GetError(hi2c1_ref1);

    uint32_t debug = 0;
}
