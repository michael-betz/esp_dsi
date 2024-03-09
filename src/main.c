#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mipi_dsi.h"
#include "oled.h"


#define LINE_LEN 640

void disp()
{
    static unsigned iter = 0;
    static uint8_t clrData[LINE_LEN * 2 + 1];

    if (iter == 0)
        clrData[0] = 0x2C;  // CMD: write from start
    else
        clrData[0] = 0x3C;  // CMD: continue write

    uint8_t *p = &clrData[1];
    for (unsigned i = 0; i < LINE_LEN; i++) {
        // native color format is RGB 565
        // here we drive it in RGB 555 to get white
        unsigned r = (i + iter / 3) & 0x1F;
        unsigned g = (i + iter / 3) & 0x1F;
        unsigned b = (i + iter / 3) & 0x1F;
        unsigned tmp = (b << 11) | (g << 6) | r;
        *p++ = tmp;
        *p++ = tmp >> 8;
    }

    printf("row %d\n", iter);

    mipiDsiSendLong(0x39, clrData, sizeof(clrData));

    iter++;
}

void app_main(void)
{
    // Initialize the DSI interface
    mipiInit();
    initOled();
    set_brightness(0xFF);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // setColRange(319 - 16 - 1, 319);
    // setRowRange(0, 320);
    // setColRange(0, 159);
    while (1) {
        disp();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
