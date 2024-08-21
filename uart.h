#ifndef uart
#define uart

#include "driver/uart.h"
#include "esp_log.h"

#define UART_BUF_SIZE (1024)
#define UART_BAUD_RATE 115200
#define UART_PORT_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)  

//void uart_data_prepare(uint16_t *data, int len);
esp_err_t uart_init(void);

#endif