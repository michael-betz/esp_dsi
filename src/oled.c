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
	// {0, 0x12, {0}},  // partial display mode
	{0, 0x13, {0}},  // normal display mode
	{0, 0x11, {0}},  // exit_sleep_mode (need to wait 5 ms now)
	// {0, 0x22, {0}},  // All pixels off
	// {0, 0x28, {0}},  // display off
	// {0, 0x29, {0}},  // display on
	// {0, 0x23, {0}},  // All pixels on
	{1, 0x3A, {0x55}},  // 16-bit / pixel mode
};


// typedef struct {
// 	uint8_t type;
// 	uint8_t addr;
// 	uint8_t len;
// 	uint8_t data[16];
// } DispPacket;

// //Copied from the X163QLN01 application note.
// static const DispPacket initPackets[]={
// 	{0x39, 0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
// 	{0x39, 0xBD, 5, {0x01, 0x90, 0x14, 0x14, 0x00}},
// 	{0x39, 0xBE, 5, {0x01, 0x90, 0x14, 0x14, 0x01}},
// 	{0x39, 0xBF, 5, {0x01, 0x90, 0x14, 0x14, 0x00}},
// 	{0x39, 0xBB, 3, {0x07, 0x07, 0x07}},
// 	{0x39, 0xC7, 1, {0x40}},
// 	{0x39, 0xF0, 5, {0x55, 0xAA, 0x52, 0x80, 0x02}},
// 	{0x39, 0xFE, 2, {0x08, 0x50}},
// 	{0x39, 0xC3, 3, {0xF2, 0x95, 0x04}},
// 	{0x15, 0xCA, 1, {0x04}},
// 	{0x39, 0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}},
// 	{0x39, 0xB0, 3, {0x03, 0x03, 0x03}},
// 	{0x39, 0xB1, 3, {0x05, 0x05, 0x05}},
// 	{0x39, 0xB2, 3, {0x01, 0x01, 0x01}},
// 	{0x39, 0xB4, 3, {0x07, 0x07, 0x07}},
// 	{0x39, 0xB5, 3, {0x05, 0x05, 0x05}},
// 	{0x39, 0xB6, 3, {0x53, 0x53, 0x53}},
// 	{0x39, 0xB7, 3, {0x33, 0x33, 0x33}},
// 	{0x39, 0xB8, 3, {0x23, 0x23, 0x23}},
// 	{0x39, 0xB9, 3, {0x03, 0x03, 0x03}},
// 	{0x39, 0xBA, 3, {0x13, 0x13, 0x13}},
// 	{0x39, 0xBE, 3, {0x22, 0x30, 0x70}},
// 	{0x39, 0xCF, 7, {0xFF, 0xD4, 0x95, 0xEF, 0x4F, 0x00, 0x04}},
// 	{0x15, 0x35, 1, {0x01}}, //
// 	{0x15, 0x36, 1, {0x00}}, //
// 	{0x15, 0xC0, 1, {0x20}}, //
// 	{0x39, 0xC2, 6, {0x17, 0x17, 0x17, 0x17, 0x17, 0x0B}},
// 	{0x32, 0, 0, {0}},
// 	{0x05, 0x11, 1, {0x00}}, //exit_sleep_mode
// 	{0x05, 0x29, 1, {0x00}}, //turn display on
// 	{0x15, 0x3A, 1, {0x55}}, //16-bit mode
// //	{0x29, 0x2B, 4, {0x00, 0x00, 0x00, 0xEF}},
// 	{0,0,0,{0}}
// };



#define N_INIT_PACKETS (sizeof(initPackets) / sizeof(initPackets[0]))

#define H_LINES 32
#define N_LINES 10
#define PX_LINE 320
#define BYTES_LINE (PX_LINE * 2)

static void cmd0(uint8_t addr)
{
	// DCS short write without parameters
	mipiDsiSendShort(0x05, &addr, 1);
}

static void cmd1(uint8_t addr, uint8_t val)
{
	// DCS short write with 1 parameter
	uint8_t data[2] = {addr, val};
	mipiDsiSendShort(0x15, data, sizeof(data));
}

static void cmdN(uint8_t addr, uint8_t *data, unsigned len)
{
	// DCS Long write
	mipiDsiSendLong(0x39, addr, data, len);
}


void initOled() {
	//Reset display
	gpio_set_level(GPIO_NRST, 0);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_NRST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	mipiResync();


	// for (int i=0; initPackets[i].type!=0; i++) {
	// 		// mipiResync();
	// 		if (initPackets[i].type==0x39 || initPackets[i].type==0x29) {
	// 			uint8_t data[17];
	// 			data[0]=initPackets[i].addr;
	// 			memcpy(data+1, initPackets[i].data, 16);
	// 			mipiDsiSendLong(initPackets[i].type, data, initPackets[i].len+1);
	// 		} else {
	// 			uint8_t data[2]={initPackets[i].addr, initPackets[i].data[0]};
	// 			mipiDsiSendShort(initPackets[i].type, data, initPackets[i].len+1);
	// 			if (initPackets[i].type==5) vTaskDelay(300/portTICK_PERIOD_MS);
	// 		}
	// }

	// uint8_t clrData[33]={0x2c, 0, 0, 0};
	// for (int i=0; i<(320*340*2)/32; i++) {
	// 	mipiDsiSendLong(0x39, clrData, 32);
	// 	clrData[0]=0x3C;
	// }


	DispPacket *p = initPackets;

	for (int i = 0; i < N_INIT_PACKETS; i++) {
		if (p->len == 0) {
			cmd0(p->addr);
			if (p->addr == 0x11)
				vTaskDelay(5 / portTICK_PERIOD_MS);
		} else if (p->len == 1) {
			cmd1(p->addr, p->data[0]);
		} else if (p->len < 8) {
			cmdN(p->addr, p->data, p->len);
		} else {
			assert(0);
		}
		p++;
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// display on
    uint8_t cmd = 0x29;
    mipiDsiSendShort(0x05, &cmd, 1);

	printf("Display inited.\n");
}

void setColRange(int xstart, int xend) {
	// set_col_addr
	uint8_t cmd[4];

	//No idea why the * 2... maybe because of 2 bytes per pixel?
	xstart = xstart * 2;
	xend = xend * 2;
	cmd[0] = (xstart >> 8); 	// scolh
	cmd[1] = (xstart & 0xff); 	// scoll
	cmd[2] = (xend >> 8); 		// ecolh
	cmd[3] = (xend & 0xff); 	// ecoll
	cmdN(0x2a, cmd, sizeof(cmd));
}

void setRowRange(int ystart, int yend) {
	// set_page_addr
	uint8_t cmd[4];
	cmd[0] = (ystart >> 8); 	// scolh
	cmd[1] = (ystart & 0xff); 	// scoll
	cmd[2] = (yend >> 8); 		// ecolh
	cmd[3] = (yend & 0xff); 	// ecoll
	cmdN(0x2b, cmd, sizeof(cmd));
}

void set_brightness(uint8_t val)
{
	cmd1(0x51, val);
}

void write_rect(unsigned x0, unsigned x1, unsigned y0, unsigned y1, uint8_t *data)
{
	setColRange(x0, x1);
	setRowRange(y0, y1);
	// 2c: start write
	// 3c: continue write
	cmdN(0x2c, data, (x1 - x0 + 1) * (y1 - y0 + 1) * 2);
}
