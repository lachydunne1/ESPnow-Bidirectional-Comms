#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "nvs_flash.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/uart_select.h"
#include "uart.h"
#include "espnow_bidir_config.h"

/* switch logical value for flashing different devices */
// DEVICE_SWITCH = 1 is MAC1 [ESP32s2]
// DEVICE_SWITCH = 0 is MAC2 [ESP32]
#define DEVICE_SWITCH 0

static const char *TAG = "espnow_bidirectional_main";
static uint8_t MAC1[ESP_NOW_ETH_ALEN] = {0x64, 0xB7, 0x08, 0xCE, 0x85, 0xA8};
static uint8_t MAC2[ESP_NOW_ETH_ALEN] = {0xD4, 0xF9, 0x8D, 0x72, 0x90, 0x60};

static esp_err_t bidirectional_init_esp_now(void);
void espnow_shutdown(void);
void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/*TODO: 
    I know UART is bugging the ESP32
    I think send.py is not sending data over uart, or uart aren't triggering.
*/
espnow_send_param_t *send_param;
void uart_data_prepare(uint8_t *mac_addr, int len);
void uart_init(void);

void espnow_data_parse(const uint8_t *mac_addr, const uint8_t *data, int len);

static QueueHandle_t event_queue;
static void queue_sm_task(void *pvParameters);

void app_main(void){

    event_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    wifi_init();
    bidirectional_init_esp_now();
    uart_init(); 

    xTaskCreate(queue_sm_task, "queue_task", 2048, NULL, 4, NULL);
}

static esp_err_t bidirectional_init_esp_now(){

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    esp_now_peer_info_t *peer_info = malloc(sizeof(esp_now_peer_info_t));
    if(peer_info==NULL){
        return ESP_FAIL;
    }
    /* initialize contiguous block of memory for peer */
    memset(peer_info, 0, sizeof(esp_now_peer_info_t));
    
    /* Add peers */
#if DEVICE_SWITCH
    memcpy(peer_info->peer_addr, MAC2, ESP_NOW_ETH_ALEN);
#else 
    memcpy(peer_info->peer_addr, MAC1, ESP_NOW_ETH_ALEN);
#endif
    peer_info->channel = CONFIG_ESPNOW_CHANNEL;
    peer_info->encrypt = false;
    /* add peer to peer list */
    ESP_ERROR_CHECK(esp_now_add_peer(peer_info));
    free(peer_info);

    return ESP_OK;
}

void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len){

    espnow_event_t event;
    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0){
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    event.id = ESPNOW_RECV_CB;
    /* copy data into event */
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);

    if(recv_cb->data == NULL){
        ESP_LOGE(TAG, "Malloc receive data fail");
        free(recv_cb->data);
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

    if(xQueueSend(event_queue, &event, CONFIG_DELAY_TIME) != pdTRUE){
        ESP_LOGE(TAG, "Failed to send receive event to queue");
        free(recv_cb->data);
    }

}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
    
    espnow_event_t event;
    espnow_event_send_cb_t *send_cb = &event.info.send_cb;

    if (mac_addr == NULL){
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    event.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;

    if(xQueueSend(event_queue, &event, CONFIG_DELAY_TIME) != pdTRUE){
        ESP_LOGE(TAG, "Failed to send receive event to queue");
    }

}

static void queue_sm_task(void *pvParameters){

    espnow_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending data");

    for (;;){
        while (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdTRUE){
                
            switch(event.id){
                case ESPNOW_RECV_CB:

                    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;
                    ESP_LOGI(TAG, "%d bytes incoming from"MACSTR"", recv_cb->data_len, MAC2STR(recv_cb->mac_addr));
                    espnow_data_parse(recv_cb->mac_addr, recv_cb->data, recv_cb-> data_len);
                    break;

                case ESPNOW_SEND_CB:

                    espnow_event_send_cb_t *send_cb = &event.info.send_cb;
                    ESP_LOGI(TAG, "Send status: %d", send_cb->status);

                    break;

                case UART_DATA_EVENT:
                    // TODO: FINISH

                    /* malloc send param space for packing in prepare_data function*/
                    send_param = malloc(sizeof(espnow_send_param_t));
                    if (send_param == NULL){
                        ESP_LOGE(TAG, "Failed to allocate memory for send parameter.");
                        vTaskDelete(NULL);
                    }

                    uart_event_t *uart_cb = &event.info.uart_cb;
                    ESP_LOGI(TAG, "UART Data: %d", uart_cb->size);
                    uart_read_bytes(UART_PORT_NUM, dtmp, uart_cb->size, portMAX_DELAY);
                    uart_data_prepare(dtmp, uart_cb->size);

                    if(esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK){
                        ESP_LOGE(TAG, "Send error");
                        espnow_shutdown();
                        vTaskDelete(NULL);
                    }

                    break;

                default:

                    ESP_LOGE(TAG, "Callback type error: %d", event.id);
                    break;

            }
        }
    }

}
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


void uart_init(){

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    /* install function args: S-O RX ring buffer, UART num, S-O TX ring buffer, EVENT Q handle and size, flag and interrupt */
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &event_queue, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
}   
/* in case of failure, it is our responsibility to free memory to avoid memory leaks.*/
void espnow_shutdown(){
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(event_queue);
    esp_now_deinit();
}


void espnow_data_parse(const uint8_t *mac_addr, const uint8_t *data, int len){

    if(len!=5){
        ESP_LOGE(TAG, "Incorrect data size. %d!=5", len);
        free(data);
    }
    ESP_LOGI(TAG, "Received data from: "MACSTR",   STEPS: %d, DIR: %d", MAC2STR(mac_addr), data[1], data[2]);
    /* i have no idea what to do with the error code lmao */
}
