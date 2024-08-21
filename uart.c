//TODO: FIGURE OUT PACKING AND SENDING
/* prepare and pack send param */
void uart_data_prepare(uint8_t *data, int len){

    if (len!=5){
        ESP_LOGE(TAG, "Invalid command length");
        return;
    }
    send_param-> unicast = true;
    send_param-> broadcast = false;
    send_param-> count = CONFIG_ESPNOW_SEND_COUNT;
    send_param-> len = CONFIG_ESPNOW_SEND_LEN;
#if DEVICE_SWITCH
    memcpy(send_param->dest_mac, MAC2, ESP_NOW_ETH_ALEN);
#else 
    memcpy(send_param->dest_mac, MAC1, ESP_NOW_ETH_ALEN);
#endif
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);

    if (send_param -> buffer == NULL){
        ESP_LOGE(TAG, "Malloc send buffer fail.");
        free(send_param);
    }

    espnow_data_t *buf = (espnow_data_t *)send_param-> buffer;
    assert(send_param->len >= sizeof(espnow_data_t));

    /* debug, not sure if explicit memory access actually accessing data */
    printf("Preparing data: %i steps in the %i direction", data[1], data[2]);
    buf-> type = ESPNOW_DATA_UNICAST;
    buf -> steps = data[1];
    buf -> dir = data[2];
    buf -> crc = data[3]; //defeats the purpose of the cyclic redundancy check [uses original transmission, instead of checking]!

}


esp_err_t uart_init(){

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    /* install function args: S-O RX ring buffer, UART num, S-O TX ring buffer, EVENT Q handle and size, flag and interrupt */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &event_queue, 0));
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    return ESP_OK
}   