#ifndef E220_H
#define E220_H

#include <main.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* E220-900T22D: M0/M1 tied LOW, AUX not connected, transparent mode.
 * Configure address, channel, air rate and key externally before use.
 */
#define E220_RX_BUFFER_SIZE 256U
#define E220_MAX_PACKET_SIZE 200U
#define E220_UART_TIMEOUT_MARGIN_MS 20U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t rx_buffer[E220_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t uart_error_count;
    volatile HAL_StatusTypeDef rx_status;
} E220_t;

/* Call once after UART initialization. Uses the UART's configured baud, 8N1. */
HAL_StatusTypeDef E220_Init(E220_t *e220, UART_HandleTypeDef *huart);

/* Each send accepts at most 200 bytes. HAL_OK means UART submission only.
 * With AUX disconnected, the caller must pace traffic to the radio's air rate.
 */
HAL_StatusTypeDef E220_Send(E220_t *e220, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef E220_SendString(E220_t *e220, const char *str);

/* Non-blocking: keep data unchanged until HAL_UART_TxCpltCallback fires. */
HAL_StatusTypeDef E220_Send_IT(E220_t *e220, const uint8_t *data, uint16_t len);

/* Forward the matching HAL UART receive/error callbacks to these functions. */
void E220_RxCallback(E220_t *e220);
void E220_ErrorCallback(E220_t *e220);

/* Single consumer in the main loop; the RX callback is the sole producer.
 * Capacity is 255 bytes; a full buffer drops the newest byte and counts it.
 * ReadByte returns zero if empty: check Available first for binary data.
 */
uint16_t E220_Available(E220_t *e220);
uint8_t E220_ReadByte(E220_t *e220);
uint16_t E220_Read(E220_t *e220, uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* E220_H */
