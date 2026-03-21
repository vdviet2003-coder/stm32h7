#include "uart_dma_lib.h"
#include <string.h>

/* ========== C?u hình kích thu?c buffer ========== */
#ifndef RX_DMA_BUF_SIZE
#define RX_DMA_BUF_SIZE     64   /* DMA circular buffer */
#endif
#ifndef RX_RB_SIZE
#define RX_RB_SIZE          128  /* Ring buffer cho d? li?u dã x? lý */
#endif
#ifndef TX_RB_SIZE
#define TX_RB_SIZE          128  /* Ring buffer cho d? li?u g?i di */
#endif

/* ========== DMA RX buffer – ph?i n?m trong vùng DMA-accessible ========== */
/* V?i STM32H7, dùng attribute d? d?t vào SRAM1 (c?n linker script h? tr?) */
#if defined(__GNUC__)
__attribute__((section(".sram1")))
#endif
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
static uint8_t rx_dma_buffer[RX_DMA_BUF_SIZE];

/* ========== Ring buffer RX ========== */
static uint8_t rx_rb_data[RX_RB_SIZE];
static volatile uint16_t rx_rb_head = 0;
static volatile uint16_t rx_rb_tail = 0;

/* ========== Ring buffer TX ========== */
static uint8_t tx_rb_data[TX_RB_SIZE];
static volatile uint16_t tx_rb_head = 0;
static volatile uint16_t tx_rb_tail = 0;
static volatile uint8_t  tx_dma_busy = 0;
static volatile uint16_t tx_dma_len = 0;

/* ========== Callback cho d? li?u RX (tùy ch?n) ========== */
static UART_RxCallback_t rx_callback = NULL;

/* Ðang ký callback */
void UART_DMA_RegisterRxCallback(UART_RxCallback_t cb)
{
    rx_callback = cb;
}

/* ========== Hàm x? lý RX: l?y d? li?u t? DMA buffer vào ring buffer ========== */
static void usart_rx_check(void)
{
    static uint16_t old_pos = 0;
    uint16_t new_pos = RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    if (new_pos != old_pos)
    {
        if (new_pos > old_pos)
        {
            /* Tru?ng h?p tuy?n tính */
            for (uint16_t i = old_pos; i < new_pos; i++)
            {
                rx_rb_data[rx_rb_head] = rx_dma_buffer[i];
                rx_rb_head = (rx_rb_head + 1) % RX_RB_SIZE;
            }
        }
        else
        {
            /* Tru?ng h?p wrap-around */
            for (uint16_t i = old_pos; i < RX_DMA_BUF_SIZE; i++)
            {
                rx_rb_data[rx_rb_head] = rx_dma_buffer[i];
                rx_rb_head = (rx_rb_head + 1) % RX_RB_SIZE;
            }
            for (uint16_t i = 0; i < new_pos; i++)
            {
                rx_rb_data[rx_rb_head] = rx_dma_buffer[i];
                rx_rb_head = (rx_rb_head + 1) % RX_RB_SIZE;
            }
        }
        old_pos = new_pos;
    }
}

/* ========== Hàm kh?i d?ng TX DMA ========== */
static uint8_t usart_start_tx_dma(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (tx_dma_busy)
    {
        __set_PRIMASK(primask);
        return 0;
    }

    /* Ki?m tra có d? li?u trong ring buffer không */
    uint16_t avail;
    if (tx_rb_head >= tx_rb_tail)
        avail = tx_rb_head - tx_rb_tail;
    else
        avail = TX_RB_SIZE - tx_rb_tail;

    if (avail == 0)
    {
        __set_PRIMASK(primask);
        return 0;
    }

    /* L?y d? dài block tuy?n tính */
    uint16_t len;
    if (tx_rb_head > tx_rb_tail)
        len = tx_rb_head - tx_rb_tail;
    else
        len = TX_RB_SIZE - tx_rb_tail;

    tx_dma_busy = 1;
    tx_dma_len = len;
    HAL_UART_Transmit_DMA(&huart1, &tx_rb_data[tx_rb_tail], len);

    __set_PRIMASK(primask);
    return 1;
}

/* ========== Hàm g?i chu?i ========== */
void UART_DMA_SendString(const char *str)
{
    while (*str)
    {
        uint16_t next = (tx_rb_head + 1) % TX_RB_SIZE;
        if (next != tx_rb_tail)  /* còn ch? */
        {
            tx_rb_data[tx_rb_head] = *str++;
            tx_rb_head = next;
        }
        else break;
    }
    usart_start_tx_dma();
}

/* ========== Hàm g?i d? li?u raw ========== */
void UART_DMA_SendData(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = (tx_rb_head + 1) % TX_RB_SIZE;
        if (next != tx_rb_tail)
        {
            tx_rb_data[tx_rb_head] = data[i];
            tx_rb_head = next;
        }
        else break;
    }
    usart_start_tx_dma();
}

/* ========== Kh?i t?o ========== */
void UART_DMA_Init(void)
{
    HAL_UART_Receive_DMA(&huart1, rx_dma_buffer, RX_DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* ========== X? lý IDLE (g?i t? USART1_IRQHandler) ========== */
void UART_DMA_IDLE_Handler(void)
{
    usart_rx_check();
}

/* ========== Callbacks HAL ========== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_rb_tail = (tx_rb_tail + tx_dma_len) % TX_RB_SIZE;
        tx_dma_busy = 0;
        tx_dma_len = 0;
        usart_start_tx_dma();
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        usart_rx_check();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        usart_rx_check();
}