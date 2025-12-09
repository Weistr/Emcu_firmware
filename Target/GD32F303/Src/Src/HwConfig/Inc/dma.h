#ifndef _DMA_H
#define _DMA_H

#include "gd32e23x_dma.h"
void dmaConfig(void);
void dmaCnt_reset(dma_channel_enum channelx,uint32_t DMA_CHCNT_VALUE);

#define DMA_CH_USART0_TX DMA_CH1
#define DMA_CH_USART0_RX DMA_CH2
#endif /* MAIN_H */
