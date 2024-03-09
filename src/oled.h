#pragma once

//IO pins
#define GPIO_D0N_LS 4
#define GPIO_D0P_LS 33
#define GPIO_D0_HS 32
#define GPIO_CLKP_LS 25
#define GPIO_CLKN_LS 27
#define GPIO_FF_NRST 14
#define GPIO_FF_CLK 12
#define GPIO_NRST 5

void initOled();

// max range is 0, 319
void setColRange(int xstart, int xend);

// max range is 0, 319
void setRowRange(int ystart, int yend);

// Anything below 100 is basically off
void set_brightness(uint8_t val);
