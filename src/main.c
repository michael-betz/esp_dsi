#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mipi_dsi.h"
#include "oled.h"

void disp(unsigned color)
{
    uint8_t data[64 * 64 * 2];
    for (unsigned i = 0; i < sizeof(data) / 2; i++) {
        data[2 * i] = color;
        data[2 * i + 1] = color >> 8;
    }
    write_rect(0, 63, 0, 63, data);
}

void app_main(void)
{
    // Initialize the DSI interface
    mipiInit();
    initOled();

    unsigned color = 0xF8F;
    while (1) {
        disp(color);
        color = ((color & 1) << 16) | (color >> 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
