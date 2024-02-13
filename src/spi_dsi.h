#pragma once

void spi_init();
void send_stuff(const uint8_t *data, int len);

void mipiDsiSendLong(uint8_t type, uint8_t *data, int len);
void mipiDsiSendShort(uint8_t type, uint8_t *data, int len);
