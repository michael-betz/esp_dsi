/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain
 * this notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */
/*
Thing to emulate single-lane MIPI using a flipflop and a bunch of resistors.
*/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "rom/ets_sys.h"
#include "soc/spi_struct.h"
#include "soc/spi_reg.h"
#include "soc/gpio_struct.h"
#include "mipi_dsi.h"
#include "hal/spi_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_private/spi_common_internal.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "oled.h"

//We need speed here!
#pragma GCC optimize ("O3")

#define HOST HSPI_HOST
#define IRQSRC ETS_SPI2_DMA_INTR_SOURCE
#define DMACH 2

#define L_IDLE 8
#define N_IDLE 3
#define N_DATA 8

#define SOTEOTWAIT() {}  // asm volatile("nop; nop; nop; nop")

static spi_dev_t *spidev = &SPI2;
static int cur_idle_desc = 0;
static lldesc_t idle_dmadesc[N_IDLE];
static lldesc_t data_dmadesc[N_DATA];
static SemaphoreHandle_t sem = NULL;


static void spidma_intr(void *arg) {
	BaseType_t xHigherPriorityTaskWoken = 0;
	spidev->dma_int_clr.val = 0xFFFFFFFF;  // clear all ints
	// Data is sent

	xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD_FROM_ISR();
}

/*
Brings up the clock and data lines to LP11, resyncs the flipflop, restarts the clock and DMA engine.
*/
void mipiResync() {
	//Get clock and data transceivers back in idle state
	gpio_set_level(GPIO_CLKN_LS, 1);
	SOTEOTWAIT();
	gpio_set_level(GPIO_CLKP_LS, 1);

	//Stop DMA transfer
	spidev->dma_conf.dma_tx_stop = 1;
	while (spidev->ext2.val != 0);

	//Clear flipflop
	gpio_set_level(GPIO_FF_NRST, 0);
	ets_delay_us(1);
	gpio_set_level(GPIO_FF_NRST, 1);

	//Clock is in LP11 now. We should go LP01, LP00 to enable HS receivers
	gpio_set_level(GPIO_CLKP_LS, 0);
	SOTEOTWAIT();
	gpio_set_level(GPIO_CLKN_LS, 0);

	cur_idle_desc=0;
	idle_dmadesc[0].qe.stqe_next=&idle_dmadesc[0];
	//Set SPI to transfer contents of idle dmadesc
	spidev->dma_conf.val |= SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST;
	spidev->dma_out_link.start=0;
	spidev->dma_in_link.start=0;
	spidev->dma_conf.val &= ~(SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST);
	spidev->dma_conf.dma_tx_stop=0;
	spidev->dma_conf.dma_continue=1;
	spidev->dma_conf.out_data_burst_en=1;
	spidev->user.usr_mosi_highpart=0;
	spidev->dma_out_link.addr=(int)(&idle_dmadesc[0]) & 0xFFFFF;
	spidev->dma_out_link.start=1;
	spidev->user.usr_mosi=1;

/* HACK for inverted clock */
	// spidev->user.usr_addr=1;
	// spidev->user1.usr_addr_bitlen=0; //1 addr bit
/* End hack */

	spidev->cmd.usr=1;
}

void mipiInit() {
	spi_bus_config_t buscfg={
		.miso_io_num=-1,
		.mosi_io_num=GPIO_D0_HS,
		.sclk_io_num=GPIO_FF_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=4096*3
	};

	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_OUTPUT,
		.pin_bit_mask=(1<<GPIO_D0N_LS)|(1LL<<GPIO_D0P_LS)|(1LL<<GPIO_D0_HS)|(1<<GPIO_CLKP_LS)|
			(1<<GPIO_CLKN_LS)|(1<<GPIO_FF_NRST)|(1<<GPIO_FF_CLK)|(1<<GPIO_NRST),
	};
	gpio_config(&io_conf);

	assert(spicommon_periph_claim(HOST, "SPI2"));
	ESP_ERROR_CHECK(spicommon_bus_initialize_io(HOST, &buscfg,  SPICOMMON_BUSFLAG_MASTER, NULL));

	uint32_t d_rx = 0, d_tx = 0;
	ESP_ERROR_CHECK(spicommon_dma_chan_alloc(HOST, DMACH, &d_tx, &d_rx));
	assert(d_tx == DMACH);

	//Set up idle dma desc
	uint8_t *idle_mem = heap_caps_malloc(L_IDLE, MALLOC_CAP_DMA);
	memset(idle_mem, 0, L_IDLE);
	for (int i = 0; i < N_IDLE; i++) {
		idle_dmadesc[i].size=L_IDLE;
		idle_dmadesc[i].length=L_IDLE;
		idle_dmadesc[i].buf=idle_mem;
		idle_dmadesc[i].eof=0;
		idle_dmadesc[i].sosf=0;
		idle_dmadesc[i].owner=1;
		idle_dmadesc[i].qe.stqe_next = &idle_dmadesc[i];
	}

	esp_intr_alloc(IRQSRC, 0, spidma_intr, NULL, NULL);

	//Reset DMA
	spidev->dma_conf.val |= SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST;
	spidev->dma_out_link.start=0;
	spidev->dma_in_link.start=0;
	spidev->dma_conf.val &= ~(SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST);

	//Reset timing
	spidev->ctrl2.val=0;
	spi_ll_master_set_clock(spidev, 80000000, 40000000, 128);

	//Configure SPI host
	spidev->ctrl.rd_bit_order=1; //LSB first
	spidev->ctrl.wr_bit_order=1;
	spidev->pin.ck_idle_edge=0;
	spidev->user.ck_out_edge=0;
	spidev->ctrl2.mosi_delay_mode=0; // add 1 to shift MIPI clock phase by 90 deg
	spidev->ctrl2.miso_delay_mode=0;
	spidev->ctrl.val &= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
	spidev->user.val &= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

	//Disable unneeded ints
	spidev->slave.val=0;
	spidev->dma_int_ena.val=0;

	//Set int on EOF
	spidev->dma_int_clr.val=0xFFFFFFFF; //clear all ints
	spidev->dma_int_ena.out_eof=1;
//	spidev->dma_int_ena.in_suc_eof=1;

	//Init GPIO to MIPI idle levels
	gpio_set_level(GPIO_D0N_LS, 1);
	gpio_set_level(GPIO_D0P_LS, 1);
	gpio_set_level(GPIO_CLKN_LS, 1);
	gpio_set_level(GPIO_CLKP_LS, 1);

	sem = xSemaphoreCreateBinary();
	mipiResync();
}

//Set up a list of dma descriptors. dmadesc is an array of descriptors. Data is the buffer to point to.
static void IRAM_ATTR spicommon_setup_dma_desc_links(lldesc_t *dmadesc, int len, const uint8_t *data, bool isrx)
{
    int n = 0;
    while (len) {
        int dmachunklen = len;
        if (dmachunklen > SPI_MAX_DMA_LEN) dmachunklen = SPI_MAX_DMA_LEN;
        if (isrx) {
            //Receive needs DMA length rounded to next 32-bit boundary
            dmadesc[n].size = (dmachunklen + 3) & (~3);
            dmadesc[n].length = (dmachunklen + 3) & (~3);
        } else {
            dmadesc[n].size = dmachunklen;
            dmadesc[n].length = dmachunklen;
        }
        dmadesc[n].buf = (uint8_t *)data;
        dmadesc[n].eof = 0;
        dmadesc[n].sosf = 0;
        dmadesc[n].owner = 1;
        dmadesc[n].qe.stqe_next = &dmadesc[n + 1];
        len -= dmachunklen;
        data += dmachunklen;
        n++;
    }
    dmadesc[n - 1].eof = 1; //Mark last DMA desc as end of stream.
    dmadesc[n - 1].qe.stqe_next = NULL;
}

static void mipiSendMultiple(uint8_t **data, int *lengths, int count) {
	if (count == 0)
		return;
	assert(data[0][0]==0xB8);

	//Set up link to new transfer
	int next_idle_desc = (cur_idle_desc==0) ? 1 : 0;

	int last = -1;
	for (int i=0; i<count; i++) {
		last++;
		spicommon_setup_dma_desc_links(&data_dmadesc[last], lengths[i], data[i], false);
		//Look for new end
		for (; data_dmadesc[last].eof!=1; last++) ;
		//Kill eof and make desc point to the next one.
		data_dmadesc[last].eof = 0;
		data_dmadesc[last].qe.stqe_next = &data_dmadesc[last + 1];
	}

	//Make very last data desc go to the runout idle desc
	data_dmadesc[last].qe.stqe_next = &idle_dmadesc[2];

	// and make the runout go to the end idle desc
	idle_dmadesc[2].qe.stqe_next = &idle_dmadesc[next_idle_desc];
	idle_dmadesc[2].eof = 1;

	//Make sure that idle desc keeps spinning
	idle_dmadesc[next_idle_desc].qe.stqe_next = &idle_dmadesc[next_idle_desc];

	gpio_set_level(GPIO_D0P_LS, 0);
	SOTEOTWAIT();
	gpio_set_level(GPIO_D0N_LS, 0);

	//Break the loop on the current idle descriptor
	idle_dmadesc[cur_idle_desc].qe.stqe_next=&data_dmadesc[0];

	//Okay, done.
	cur_idle_desc=next_idle_desc; //for next time

	//Wait until transmission is done
	xSemaphoreTake(sem, portMAX_DELAY);

	gpio_set_level(GPIO_D0N_LS, 1);
	SOTEOTWAIT();
	gpio_set_level(GPIO_D0P_LS, 1);
}

static void mipiSend(uint8_t *data, int len) {
	mipiSendMultiple(&data, &len, 1);
}


// ------------------------------
//  DSI stuff
// ------------------------------
#define NO_CRC 1

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

//Warning: CRC isn't tested (my display does not use it)
void mipiDsiSendLong(uint8_t type, uint8_t *data, int len) {
	DsiLPHdr p;
	uint8_t footer[3];
	uint8_t *datas[3] = {(uint8_t*)&p, data, footer};
	int lengths[3] = {sizeof(DsiLPHdr), len, sizeof(footer)};

	p.sot = 0xB8;
	p.datatype = type;
	p.wordcount = mipiword(len);
	p.ecc = calc_ecc((uint8_t*)&p.datatype);

#if NO_CRC
	footer[0] = 0;
	footer[1] = 0;
	footer[2] = 0xff;  // need one last level transition at end
#else
	int crc=crc16_ccitt(0xFFFF, data, len);
	footer[0] = (crc & 0xff);
	footer[1] = (crc >> 8);
	footer[2] = (crc & 0x8000) ? 0 : 0xff; //need one last level transition at end
#endif

	mipiSendMultiple(datas, lengths, 3);
}

void mipiDsiSendShort(uint8_t type, uint8_t *data, int len) {
	DsiSPHdr *p = alloca(sizeof(DsiSPHdr) + 2 + len);
	p->sot = 0xB8;
	p->datatype = type;
	memcpy(p->data, data, len);
	p->data[len] = calc_ecc((uint8_t*)&p->datatype);
	p->data[len + 1] = (p->data[len] & 0x80) ? 0 : 0xff;
	mipiSend((uint8_t*)p, sizeof(DsiSPHdr) + len + 2);
}
