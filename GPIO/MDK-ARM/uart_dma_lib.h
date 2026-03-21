#ifndef UART_DMA_LIB_H
#define UART_DMA_LIB_H

#include "usart.h"  // ch?a huart1, hdma_usart1_rx, hdma_usart1_tx

#ifdef __cplusplus
extern "C" {
#endif

/* Kh?i t?o UART DMA: b?t d?u RX và b?t IDLE interrupt */
void UART_DMA_Init(void);

/* G?i chu?i k?t thúc null (ví d? "Hello\r\n") */
void UART_DMA_SendString(const char *str);

/* G?i d? li?u raw */
void UART_DMA_SendData(uint8_t *data, uint16_t len);

/* Hàm x? lý IDLE – g?i t? USART1_IRQHandler */
void UART_DMA_IDLE_Handler(void);

/* (Tùy ch?n) Ðang ký callback cho d? li?u nh?n du?c, n?u mu?n x? lý ngay */
typedef void (*UART_RxCallback_t)(uint8_t *data, uint16_t len);
void UART_DMA_RegisterRxCallback(UART_RxCallback_t cb);

#ifdef __cplusplus
}
#endif

#endif