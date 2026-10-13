/* Run against Core/Src/e220.c with tests/e220_hal ahead of Core/Inc. */
#include "e220.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static HAL_StatusTypeDef receive_result;
static HAL_StatusTypeDef transmit_result;
static HAL_StatusTypeDef transmit_it_result;
static HAL_StatusTypeDef abort_result;
static unsigned receive_calls;
static unsigned transmit_calls;
static unsigned transmit_it_calls;
static unsigned abort_calls;
static UART_HandleTypeDef *last_uart;
static uint8_t *last_rx_data;
static const uint8_t *last_tx_data;
static uint16_t last_rx_len;
static uint16_t last_tx_len;
static uint32_t last_timeout;

static void reset_mocks(void)
{
    receive_result = HAL_OK;
    transmit_result = HAL_OK;
    transmit_it_result = HAL_OK;
    abort_result = HAL_OK;
    receive_calls = transmit_calls = transmit_it_calls = abort_calls = 0U;
    last_uart = NULL;
    last_rx_data = NULL;
    last_tx_data = NULL;
    last_rx_len = last_tx_len = 0U;
    last_timeout = 0U;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                    uint8_t *data, uint16_t len)
{
    ++receive_calls;
    last_uart = huart;
    last_rx_data = data;
    last_rx_len = len;
    return receive_result;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                  const uint8_t *data, uint16_t len,
                                  uint32_t timeout)
{
    ++transmit_calls;
    last_uart = huart;
    last_tx_data = data;
    last_tx_len = len;
    last_timeout = timeout;
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    return transmit_result;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart,
                                     const uint8_t *data, uint16_t len)
{
    ++transmit_it_calls;
    last_uart = huart;
    last_tx_data = data;
    last_tx_len = len;
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    return transmit_it_result;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    ++abort_calls;
    last_uart = huart;
    return abort_result;
}

HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart)
{
    return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

static UART_HandleTypeDef normal_uart(uint32_t baud)
{
    UART_HandleTypeDef uart;
    memset(&uart, 0, sizeof(uart));
    uart.Instance = (void *)1;
    uart.Init.BaudRate = baud;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.gState = HAL_UART_STATE_READY;
    uart.RxState = HAL_UART_STATE_READY;
    return uart;
}

static void init_radio(E220_t *radio, UART_HandleTypeDef *uart)
{
    reset_mocks();
    memset(radio, 0xA5, sizeof(*radio));
    assert(E220_Init(radio, uart) == HAL_OK);
    assert(radio->huart == uart);
    assert(radio->rx_head == 0U);
    assert(radio->rx_tail == 0U);
    assert(radio->rx_overflow_count == 0U);
    assert(radio->uart_error_count == 0U);
    assert(radio->rx_status == HAL_OK);
    assert(receive_calls == 1U);
    assert(last_uart == uart);
    assert(last_rx_data == &radio->rx_byte);
    assert(last_rx_len == 1U);
}

static void inject_byte(E220_t *radio, uint8_t byte)
{
    unsigned before = receive_calls;
    radio->rx_byte = byte;
    E220_RxCallback(radio);
    assert(receive_calls == before + 1U);
    assert(last_rx_data == &radio->rx_byte);
    assert(last_rx_len == 1U);
}

static void test_initialization(void)
{
    static const uint32_t supported[] = {
        1200U, 2400U, 4800U, 9600U, 19200U, 38400U, 57600U, 115200U
    };
    E220_t radio;
    UART_HandleTypeDef uart;
    for (unsigned i = 0; i < sizeof(supported) / sizeof(supported[0]); ++i) {
        uart = normal_uart(supported[i]);
        init_radio(&radio, &uart);
    }

    reset_mocks();
    uart = normal_uart(9600U);
    assert(E220_Init(NULL, &uart) == HAL_ERROR);
    assert(E220_Init(&radio, NULL) == HAL_ERROR);
    assert(receive_calls == 0U);

    uart.Instance = NULL;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart = normal_uart(9600U);
    uart.Init.BaudRate = 0U;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart.Init.BaudRate = 230400U;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart = normal_uart(9600U);
    uart.Init.WordLength = UART_WORDLENGTH_9B;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart = normal_uart(9600U);
    uart.Init.Parity = UART_PARITY_EVEN;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart = normal_uart(9600U);
    uart.Init.StopBits = UART_STOPBITS_2;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    uart = normal_uart(9600U);
    uart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    assert(receive_calls == 0U);

    uart = normal_uart(9600U);
    receive_result = HAL_ERROR;
    assert(E220_Init(&radio, &uart) == HAL_ERROR);
    assert(receive_calls == 1U);
    assert(radio.rx_status == HAL_ERROR);
    receive_result = HAL_BUSY;
    assert(E220_Init(&radio, &uart) == HAL_BUSY);
    assert(receive_calls == 2U);
    assert(radio.rx_status == HAL_BUSY);
}

static void test_blocking_transmit(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    uint8_t payload[201];
    memset(payload, 0x5A, sizeof(payload));
    init_radio(&radio, &uart);

    assert(E220_Send(&radio, payload, 200U) == HAL_OK);
    assert(transmit_calls == 1U);
    assert(last_uart == &uart);
    assert(last_tx_data == payload);
    assert(last_tx_len == 200U);
    /* 200 * 10 bits / 9600 baud = 208.333 ms, rounded up + 20 ms. */
    assert(last_timeout == 229U);

    uart.Init.BaudRate = 1200U;
    assert(E220_Send(&radio, payload, 200U) == HAL_OK);
    assert(last_timeout == 1687U);
    uart.Init.BaudRate = 115200U;
    assert(E220_Send(&radio, payload, 1U) == HAL_OK);
    assert(last_timeout == 21U);

    unsigned before = transmit_calls;
    assert(E220_Send(&radio, NULL, 0U) == HAL_OK);
    assert(E220_Send(&radio, payload, 0U) == HAL_OK);
    assert(transmit_calls == before);
    assert(E220_Send(NULL, payload, 1U) == HAL_ERROR);
    assert(E220_Send(&radio, NULL, 1U) == HAL_ERROR);
    assert(E220_Send(&radio, payload, 201U) == HAL_ERROR);
    E220_t detached = {0};
    assert(E220_Send(&detached, payload, 1U) == HAL_ERROR);
    assert(E220_Send(&detached, NULL, 0U) == HAL_ERROR);
    assert(transmit_calls == before);
    uart.Init.BaudRate = 0U;
    assert(E220_Send(&radio, payload, 1U) == HAL_ERROR);
    assert(E220_Send_IT(&radio, payload, 1U) == HAL_ERROR);
    assert(transmit_calls == before);
    assert(transmit_it_calls == 0U);
    uart.Init.BaudRate = 9600U;

    uart.gState = HAL_UART_STATE_BUSY_TX;
    assert(E220_Send(&radio, payload, 1U) == HAL_BUSY);
    uart.gState = HAL_UART_STATE_READY;
    transmit_result = HAL_TIMEOUT;
    assert(E220_Send(&radio, payload, 1U) == HAL_TIMEOUT);
    transmit_result = HAL_ERROR;
    assert(E220_Send(&radio, payload, 1U) == HAL_ERROR);
}

static void test_strings_and_interrupt_transmit(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    uint8_t payload[201] = {1U, 2U, 3U};
    char string[202];
    init_radio(&radio, &uart);
    memset(string, 'x', 200U);
    string[200] = '\0';
    assert(E220_SendString(&radio, string) == HAL_OK);
    assert(last_tx_len == 200U);
    string[200] = 'x';
    string[201] = '\0';
    unsigned before = transmit_calls;
    assert(E220_SendString(&radio, string) == HAL_ERROR);
    assert(E220_SendString(&radio, NULL) == HAL_ERROR);
    assert(E220_SendString(NULL, "hello") == HAL_ERROR);
    assert(E220_SendString(&radio, "") == HAL_OK);
    assert(transmit_calls == before);

    assert(E220_Send_IT(&radio, payload, 3U) == HAL_OK);
    assert(transmit_it_calls == 1U);
    assert(last_tx_data == payload);
    assert(last_tx_len == 3U);
    /* HAL uses the caller's buffer until its completion callback. */
    payload[0] = 42U;
    assert(last_tx_data[0] == 42U);
    before = transmit_it_calls;
    assert(E220_Send_IT(&radio, NULL, 0U) == HAL_OK);
    assert(E220_Send_IT(&radio, NULL, 1U) == HAL_ERROR);
    assert(E220_Send_IT(NULL, payload, 1U) == HAL_ERROR);
    assert(E220_Send_IT(&radio, payload, 201U) == HAL_ERROR);
    E220_t detached = {0};
    assert(E220_Send_IT(&detached, payload, 1U) == HAL_ERROR);
    assert(transmit_it_calls == before);
    uart.gState = HAL_UART_STATE_BUSY_TX;
    assert(E220_Send_IT(&radio, payload, 3U) == HAL_BUSY);
    uart.gState = HAL_UART_STATE_READY;
    transmit_it_result = HAL_ERROR;
    assert(E220_Send_IT(&radio, payload, 3U) == HAL_ERROR);
}

static void test_receive_fifo_wrap_and_overflow(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    uint8_t out[256];
    init_radio(&radio, &uart);
    assert(E220_Available(&radio) == 0U);
    assert(E220_ReadByte(&radio) == 0U);
    for (unsigned i = 0; i < 250U; ++i) {
        inject_byte(&radio, (uint8_t)i);
    }
    assert(E220_Available(&radio) == 250U);
    assert(E220_Read(&radio, out, 200U) == 200U);
    for (unsigned i = 0; i < 200U; ++i) {
        assert(out[i] == (uint8_t)i);
    }
    for (unsigned i = 250U; i < 450U; ++i) {
        inject_byte(&radio, (uint8_t)i);
    }
    assert(E220_Available(&radio) == 250U);
    assert(E220_Read(&radio, out, sizeof(out)) == 250U);
    for (unsigned i = 0; i < 250U; ++i) {
        assert(out[i] == (uint8_t)(200U + i));
    }
    assert(E220_Available(&radio) == 0U);
    assert(radio.rx_overflow_count == 0U);

    init_radio(&radio, &uart);
    for (unsigned i = 0; i < 255U; ++i) {
        inject_byte(&radio, (uint8_t)i);
    }
    assert(E220_Available(&radio) == 255U);
    inject_byte(&radio, 0xF1U);
    inject_byte(&radio, 0xF2U);
    inject_byte(&radio, 0xF3U);
    assert(radio.rx_overflow_count == 3U);
    assert(E220_Available(&radio) == 255U);
    assert(E220_Read(&radio, out, sizeof(out)) == 255U);
    for (unsigned i = 0; i < 255U; ++i) {
        assert(out[i] == (uint8_t)i);
    }
    inject_byte(&radio, 0xB7U);
    assert(E220_ReadByte(&radio) == 0xB7U);
    assert(E220_Available(&radio) == 0U);
}

static void test_uart_error_recovery(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    init_radio(&radio, &uart);
    inject_byte(&radio, 0x31U);
    unsigned before = receive_calls;
    uart.ErrorCode = HAL_UART_ERROR_ORE;
    E220_ErrorCallback(&radio);
    assert(radio.uart_error_count == 1U);
    assert(radio.rx_status == HAL_OK);
    assert(abort_calls == 1U);
    assert(receive_calls == before + 1U);
    assert(last_rx_data == &radio.rx_byte);
    /* Recovering the UART preserves already received application bytes. */
    assert(E220_ReadByte(&radio) == 0x31U);

    inject_byte(&radio, 0x32U);
    abort_result = HAL_ERROR;
    before = receive_calls;
    E220_ErrorCallback(&radio);
    assert(radio.uart_error_count == 2U);
    assert(radio.rx_status == HAL_ERROR);
    assert(abort_calls == 2U);
    assert(receive_calls == before);
    assert(E220_ReadByte(&radio) == 0x32U);
}

static void test_receive_rearm_failure(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    init_radio(&radio, &uart);
    receive_result = HAL_ERROR;
    inject_byte(&radio, 0x6CU);
    assert(radio.rx_status == HAL_ERROR);
    unsigned before = receive_calls;
    E220_ErrorCallback(&radio);
    assert(abort_calls == 1U);
    assert(receive_calls == before + 1U);
    assert(radio.rx_status == HAL_ERROR);
    assert(radio.uart_error_count == 1U);
    assert(E220_ReadByte(&radio) == 0x6CU);
    receive_result = HAL_OK;
    E220_ErrorCallback(&radio);
    assert(radio.rx_status == HAL_OK);
    assert(radio.uart_error_count == 2U);
    inject_byte(&radio, 0x2DU);
    assert(E220_ReadByte(&radio) == 0x2DU);
}

static void test_receive_invalid_arguments(void)
{
    UART_HandleTypeDef uart = normal_uart(9600U);
    E220_t radio;
    uint8_t out;
    init_radio(&radio, &uart);
    inject_byte(&radio, 0xA3U);
    assert(E220_Available(NULL) == 0U);
    assert(E220_ReadByte(NULL) == 0U);
    assert(E220_Read(NULL, &out, 1U) == 0U);
    assert(E220_Read(&radio, NULL, 1U) == 0U);
    assert(E220_Read(&radio, &out, 0U) == 0U);
    assert(E220_Available(&radio) == 1U);
    unsigned before = receive_calls;
    unsigned abort_before = abort_calls;
    E220_RxCallback(NULL);
    E220_ErrorCallback(NULL);
    E220_t detached = {0};
    E220_RxCallback(&detached);
    E220_ErrorCallback(&detached);
    assert(receive_calls == before);
    assert(abort_calls == abort_before);
}

int main(void)
{
    test_initialization();
    test_blocking_transmit();
    test_strings_and_interrupt_transmit();
    test_receive_fifo_wrap_and_overflow();
    test_uart_error_recovery();
    test_receive_rearm_failure();
    test_receive_invalid_arguments();
    puts("E220 HAL-mocked tests passed");
    return 0;
}
