#pragma once

//IO pins
#define GPIO_D0N_LS 4
#define GPIO_D0P_LS 33
#define GPIO_D0N_HS 36
#define GPIO_D0P_HS 32
#define GPIO_CLKN_LS 27
#define GPIO_CLKP_LS 25
#define GPIO_FF_NRST 14
#define GPIO_FF_CLK 12
#define GPIO_NRST 5

void initOled();
void set_brightness(uint8_t val);
void write_rect(unsigned x0, unsigned x1, unsigned y0, unsigned y1, uint8_t *data);
