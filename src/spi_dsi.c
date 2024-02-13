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
#include "crc16-ccitt.h"

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

#define SOTEOTWAIT() asm volatile("nop;")


spi_device_handle_t spi;


void send_stuff(const uint8_t *data, int len)
{
    spi_transaction_t t = {0};
    if (len == 0)
        return;

    t.length = len * 8;     //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;     //Data

    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));  //Transmit (blocking)
    // ESP_ERROR_CHECK(spi_device_queue_trans(spi, &t, portMAX_DELAY));  // Transmit (non-blocking)
}


//This function is called (in irq context!) just before a transmission starts.
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
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
}

//This function is called (in irq context!) just after a transmission ends.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t)
{
    //Get clock and data transceivers back in idle state
    gpio_set_level(GPIO_D0N_LS, 1);
    gpio_set_level(GPIO_D0P_LS, 1);

    // TODO: disable HS clock output now!

    gpio_set_level(GPIO_CLKN_LS, 1);
    gpio_set_level(GPIO_CLKP_LS, 1);
}

//Initialize the display
void spi_init()
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
        .clock_speed_hz=40*1000*1000,           // Clock out at 80 MHz
        .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_NO_RETURN_RESULT | SPI_DEVICE_BIT_LSBFIRST,
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // CS pin
        .dummy_bits=32,                         // Clock cycles with 0 data before the payload
        .queue_size=7,                          // We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line
        .post_cb=lcd_spi_post_transfer_callback
    };

    //Initialize the SPI bus and attach the device
    ESP_ERROR_CHECK(spi_bus_initialize(DSI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(DSI_HOST, &devcfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

     //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {0};
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


//Reminder; MIPI is very LSB-first.
typedef struct {
    uint8_t sot; //should be 0xB8
    uint8_t datatype;
    uint16_t wordcount;
    uint8_t ecc; //0
    uint8_t data[];
    //Footer is 2 bytes of checksum.
} __attribute__((packed)) DsiLPHdr;


typedef struct {
    uint8_t sot; //should be 0xB8
    uint8_t datatype;
    uint8_t data[];
    //Footer is 1 byte of ECC
} __attribute__((packed)) DsiSPHdr;


// calc XOR parity bit of all '1's in 24 bit value
static char parity (uint32_t val) {
    uint32_t i,p;
    p=0;
    for(i=0; i!=24; i++) {
        p^=val;
        val>>=1;
    }
    return (p&1);
}

static uint8_t calc_ecc(uint8_t *buf) {
    int ret=0;
    uint32_t cmd=(buf[0])+(buf[1]<<8)+(buf[2]<<16);
    if(parity(cmd & 0b111100010010110010110111)) ret|=0x01;
    if(parity(cmd & 0b111100100101010101011011)) ret|=0x02;
    if(parity(cmd & 0b011101001001101001101101)) ret|=0x04;
    if(parity(cmd & 0b101110001110001110001110)) ret|=0x08;
    if(parity(cmd & 0b110111110000001111110000)) ret|=0x10;
    if(parity(cmd & 0b111011111111110000000000)) ret|=0x20;
    return ret;
}

static uint16_t mipiword(uint16_t val) {
    uint16_t ret;
    uint8_t *pret=(uint8_t*)&ret;
    pret[0]=(val&0xff);
    pret[1]=(val>>8);
    return ret;
}

#define N_PADDING 32

// Warning: CRC isn't tested (my display does not use it)
void mipiDsiSendLong(uint8_t type, uint8_t *data, int len) {
    unsigned n_bytes = sizeof(DsiLPHdr) + len + 3 + N_PADDING;
    DsiLPHdr *p = alloca(n_bytes);

    p->sot = 0xB8;
    p->datatype = type;
    p->wordcount = mipiword(len);
    p->ecc = calc_ecc((uint8_t*)&p->datatype);

    memcpy(p->data, data, len);

#if NO_CRC
    p->data[len] = 0x00;
    p->data[len + 1] = 0x00;
    p->data[len + 2] = 0xff;
#else
    int crc = crc16_ccitt(0xFFFF, data, len);
    p->data[len] = crc & 0xff;
    p->data[len + 1] = crc >> 8;
    p->data[len + 2] = (crc&0x8000) ? 0 : 0xff;  // need one last level transition at end
#endif

    memset(&p->data[len + 3], p->data[len + 2], N_PADDING);

    send_stuff((uint8_t*)p, n_bytes);
}

void mipiDsiSendShort(uint8_t type, uint8_t *data, int len) {
    unsigned n_bytes = sizeof(DsiSPHdr) + len + 2 + N_PADDING;
    DsiSPHdr *p = alloca(n_bytes);

    p->sot = 0xB8;
    p->datatype = type;

    memcpy(p->data, data, len);

    p->data[len] = calc_ecc((uint8_t*)&p->datatype);
    p->data[len + 1] = (p->data[len] & 0x80) ? 0 : 0xff;  // need one last level transition at end

    memset(&p->data[len + 2], p->data[len + 1], N_PADDING);

    send_stuff((uint8_t*)p, n_bytes);
}
