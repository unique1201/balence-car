#ifndef __MASTER_H
#define __MASTER_H



void SPI1_Master_Init(void);
uint8_t SPI1_Master_SendByte(uint8_t data);
void SPI1_Master_Transmit(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size);





#endif
