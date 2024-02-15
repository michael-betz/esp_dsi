#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mipi_dsi.h"
#include "oled.h"


#define N_LINES 64

void disp(unsigned col)
{
    uint8_t clrData[N_LINES + 1];
    for (unsigned i = 0; i < sizeof(clrData) / 2; i++) {
        clrData[2 * i] = col;
        clrData[2 * i + 1] = col >> 8;
    }
    clrData[0] = 0x3C;
    for (int i=0; i < (320 * 340 * 2) / N_LINES; i++) {
        mipiDsiSendLong(0x39, clrData, N_LINES);
    }
}

void app_main(void)
{
    // Initialize the DSI interface
    mipiInit();
    initOled();

    unsigned col = 0xF8F;
    while (1) {
        disp(col);
        col = ((col & 1) << 16) | (col >> 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
