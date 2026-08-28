// #ifndef pca9685
// #define pca9685

#include <stdint.h>

void pca9685_setup(I2C_HandleTypeDef *hi2c1_ref);
void set_channel_ticks(uint16_t off[16], uint16_t on[16]);

// #endif