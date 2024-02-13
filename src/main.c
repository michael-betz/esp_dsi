#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_dsi.h"
#include "oled_dsi.h"


void app_main(void)
{
    // Initialize the DSI interface
    spi_init();

    // uint8_t d[] = {0x12, 0x34, 0x56, 0x78};
    // uint8_t e[] = {0xFF, 0x00, 0xFF, 0x11};
    // uint8_t f[] = {0x01, 0x02, 0x03};

    while (1) {
        // send_stuff(d, sizeof(d));
        // send_stuff(e, sizeof(e));
        // send_stuff(f, sizeof(f));
        initOled();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
