/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain
 * this notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */
#include <stdio.h>
#include <alloca.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mipi_dsi.h"
#include "driver/gpio.h"
#include "oled.h"

typedef struct {
	uint8_t len;
	uint8_t addr;
	uint8_t data[8];
} DispPacket;

//Copied from the X163QLN01 application note.
static DispPacket initPackets[] = {
	{5, 0xF0, {0x55, 0xAA, 0x52, 0x08, 0x00}},
	{5, 0xBD, {0x01, 0x90, 0x14, 0x14, 0x00}},
	{5, 0xBE, {0x01, 0x90, 0x14, 0x14, 0x01}},
	{5, 0xBF, {0x01, 0x90, 0x14, 0x14, 0x00}},
	{3, 0xBB, {0x07, 0x07, 0x07}},
	{1, 0xC7, {0x40}},
	{5, 0xF0, {0x55, 0xAA, 0x52, 0x80, 0x02}},
	{2, 0xFE, {0x08, 0x50}},
	{3, 0xC3, {0xF2, 0x95, 0x04}},
	{1, 0xCA, {0x04}},
	{5, 0xF0, {0x55, 0xAA, 0x52, 0x08, 0x01}},
	{3, 0xB0, {0x03, 0x03, 0x03}},
	{3, 0xB1, {0x05, 0x05, 0x05}},
	{3, 0xB2, {0x01, 0x01, 0x01}},
	{3, 0xB4, {0x07, 0x07, 0x07}},
	{3, 0xB5, {0x05, 0x05, 0x05}},
	{3, 0xB6, {0x53, 0x53, 0x53}},
	{3, 0xB7, {0x33, 0x33, 0x33}},
	{3, 0xB8, {0x23, 0x23, 0x23}},
	{3, 0xB9, {0x03, 0x03, 0x03}},
	{3, 0xBA, {0x13, 0x13, 0x13}},
	{3, 0xBE, {0x22, 0x30, 0x70}},
	{7, 0xCF, {0xFF, 0xD4, 0x95, 0xEF, 0x4F, 0x00, 0x04}},
	{1, 0x35, {0x01}},
	{1, 0x36, {0x00}},
	{1, 0xC0, {0x20}},
	{6, 0xC2, {0x17, 0x17, 0x17, 0x17, 0x17, 0x0B}},
	{1, 0x3A, {0x55}},  // 16-bit mode
	{0, 0x13, {0}},  // normal display mode
	// {0, 0x22, {0}},  // All pixels off
	// {0, 0x23, {0}},  // All pixels on
	// {0, 0x28, {0}},  // display off
	// {1, 0x66, {0x2}},  // enable high brightness mode (no effect)
	// {1, 0x58, {0x6}},  // enable sunlight readable mode (no effect)
	// {1, 0x55, {0x3}},  // enable auto current limit (no effect)
	{0, 0x29, {0}},  // display on
	{0, 0x11, {0}},  // exit_sleep_mode (need to wait 5 ms now)
};

#define N_INIT_PACKETS (sizeof(initPackets) / sizeof(initPackets[0]))

#define N_LINES 32

void initOled() {
	//Reset display
	gpio_set_level(GPIO_NRST, 0);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_NRST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	mipiResync();

	DispPacket *p = initPackets;

	for (int i = 0; i < N_INIT_PACKETS; i++) {
		if (p->len == 0)  // DCS short write without parameters
			mipiDsiSendShort(0x05, &p->addr, 1);
		else if (p->len == 1)  // DCS short write with 1 parameter
			mipiDsiSendShort(
				0x15,  // p->addr == 0xC7 ? 0x39 : 0x15,  // maybe not needed
				&p->addr,
				2
			);
		else if (p->len < 8)  // DCS Long write
			mipiDsiSendLong(0x39, &p->addr, p->len + 1);
		else
			assert(0);
		p++;
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// uint8_t clrData[N_LINES + 1] = {0x2c, 0, 0, 0};
	// for (int i=0; i < (320 * 340 * 2) / N_LINES; i++) {
	// 	mipiDsiSendLong(0x39, clrData, N_LINES);
	// 	clrData[0] = 0x3C;
	// }

	printf("Display inited.\n");
}

void setColRange(int xstart, int xend) {
	uint8_t cmd[5];

	// From X163QLN01 app note:
	// Memory Addres of Z2 column has to be offset (+160) if the start column (SC) or end
	// column address (EC) is greater than 160.
	if (xstart > 160)
		xstart += 160;
	if (xend > 160)
		xend += 160;

	cmd[0] = 0x2a; 				// set_col_addr
	cmd[1] = (xstart >> 8); 	// scolh
	cmd[2] = (xstart & 0xff); 	// scoll
	cmd[3] = (xend >> 8); 		// ecolh
	cmd[4] = (xend & 0xff); 	// ecoll
	mipiDsiSendLong(0x39, cmd, sizeof(cmd));
}

void setRowRange(int ystart, int yend) {
	uint8_t cmd[5];
	cmd[0] = 0x2b; 				// set_page_addr
	cmd[1] = (ystart >> 8); 	// scolh
	cmd[2] = (ystart & 0xff); 	// scoll
	cmd[3] = (yend >> 8); 		// ecolh
	cmd[4] = (yend & 0xff); 	// ecoll
	mipiDsiSendLong(0x39, cmd, sizeof(cmd));
}

void set_brightness(uint8_t val)
{
	uint8_t cmd[] = {0x51, val};  // normal: 0x51, high brightness mode: 0x63
	mipiDsiSendShort(0x15, cmd, 2);
}
