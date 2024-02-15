#pragma once

void mipiInit(void);
void mipiResync(void);
void mipiDsiSendShort(uint8_t type, uint8_t *data, int len);
void mipiDsiSendLong(uint8_t type, uint8_t *data, int len);
