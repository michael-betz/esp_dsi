#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

//IO pins
#define GPIO_D0N_LS 4
#define GPIO_D0P_LS 33
#define GPIO_D0_HS 32
#define GPIO_CLKP_LS 25
#define GPIO_CLKN_LS 27
#define GPIO_FF_NRST 14
#define GPIO_FF_CLK 12
#define GPIO_NRST 5

#define DSI_HOST HSPI_HOST

#define N_PIXELS_X 320
#define PARALLEL_LINES 32

#define SOTEOTWAIT() asm volatile("nop; nop; nop;")

void send_stuff(spi_device_handle_t spi, const uint8_t *data, int len)
{
    spi_transaction_t t;
    if (len == 0)
        return;

    t.length = len * 8;     //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;     //Data

    // ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));  //Transmit (blocking)
    ESP_ERROR_CHECK(spi_device_queue_trans(spi, &t, portMAX_DELAY));  // Transmit (non-blocking)
}


//This function is called (in irq context!) just before a transmission starts.
void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    // Clear flipflop
    gpio_set_level(GPIO_FF_NRST, 0);
    SOTEOTWAIT();
    gpio_set_level(GPIO_FF_NRST, 1);

    //Clock is in LP11 now. We should go LP01, LP00 to enable HS receivers
    gpio_set_level(GPIO_CLKP_LS, 0);
    SOTEOTWAIT();
    gpio_set_level(GPIO_CLKN_LS, 0);
    SOTEOTWAIT();

    // TODO: enable HS clock output now!

    gpio_set_level(GPIO_D0P_LS, 0);
    SOTEOTWAIT();
    gpio_set_level(GPIO_D0N_LS, 0);
    SOTEOTWAIT();
}

//This function is called (in irq context!) just after a transmission ends.
void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t)
{
    //Get clock and data transceivers back in idle state
    gpio_set_level(GPIO_D0N_LS, 1);
    SOTEOTWAIT();
    gpio_set_level(GPIO_D0P_LS, 1);
    SOTEOTWAIT();

    // TODO: disable HS clock output now!

    gpio_set_level(GPIO_CLKN_LS, 1);
    SOTEOTWAIT();
    gpio_set_level(GPIO_CLKP_LS, 1);
    SOTEOTWAIT();
}

//Initialize the display
void dsi_init(spi_device_handle_t *spi)
{
    spi_bus_config_t buscfg = {
        .miso_io_num=-1,
        .mosi_io_num=GPIO_D0_HS,
        .sclk_io_num=GPIO_FF_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES * N_PIXELS_X * 2 + 8
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz=80*1000*1000,           // Clock out at 80 MHz
        .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_NO_RETURN_RESULT,
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // CS pin
        .dummy_bits=32,                         // Clock cycles with 0 data before the payload
        .queue_size=7,                          // We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line
        .post_cb=lcd_spi_post_transfer_callback
    };

    //Initialize the SPI bus and attach the device
    ESP_ERROR_CHECK(spi_bus_initialize(DSI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(DSI_HOST, &devcfg, spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(*spi, portMAX_DELAY));

     //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = \
        (1ll << GPIO_D0N_LS) | \
        (1ll << GPIO_D0P_LS) | \
        (1ll << GPIO_CLKP_LS) | \
        (1ll << GPIO_CLKN_LS) | \
        (1ll << GPIO_FF_NRST) | \
        (1ll << GPIO_NRST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);

    //Init GPIO to MIPI idle levels
    gpio_set_level(GPIO_D0N_LS, 1);
    gpio_set_level(GPIO_D0P_LS, 1);
    gpio_set_level(GPIO_CLKN_LS, 1);
    gpio_set_level(GPIO_CLKP_LS, 1);

    //Reset display
    gpio_set_level(GPIO_NRST, 0);
    ets_delay_us(200);
    gpio_set_level(GPIO_NRST, 1);
    //Wait till display lives
    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void app_main(void)
{
    spi_device_handle_t spi;

    // Initialize the DSI interface
    dsi_init(&spi);

    uint8_t d[] = {0x12, 0x34, 0x56, 0x78};
    while (1) {
        send_stuff(spi, d, sizeof(d));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
