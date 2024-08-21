#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/uart_select.h"
#include "uart.h"
#include "espnow_bidir_config.h"

/* switch logical value for flashing different devices */
// DEVICE_SWITCH = 0 is MAC1 [ESP32] sets MAC2 as PEER/RECEIVER
// DEVICE_SWITCH = 1 is MAC2 [ESP32s2] sets MAC1 as PEER/RECEIVER
#define DEVICE_SWITCH 1

static const char *TAG = "espnow_bidirectional_main";
static uint8_t MAC1[ESP_NOW_ETH_ALEN] = {0x64, 0xB7, 0x08, 0xCE, 0x85, 0xA8};
static uint8_t MAC2[ESP_NOW_ETH_ALEN] = {0xD4, 0xF9, 0x8D, 0x72, 0x90, 0x60};
//static uint8_t MAC3[ESP_NOW_ETH_ALEN] = {0xc0, 0x49, 0xef, 0xd4, 0xef, 0xc4};


static esp_err_t bidirectional_init_esp_now(void);
void espnow_shutdown(void);
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/*TODO: 
    I know UART is bugging the ESP32
    I think send.py is not sending data over uart, or uart aren't triggering.
*/
espnow_send_param_t *send_param;
void uart_data_prepare(uint8_t *mac_addr, int len);
esp_err_t uart_init(void);

void espnow_data_parse(const uint8_t *mac_addr, const uint8_t *data, int len);

static QueueHandle_t event_queue;
static QueueHandle_t uart_event_queue;

static void queue_sm_task(void *pvParameters);
static void uart_event_task(void *pvParameters);

//TODO: Find out why esp32 isnt receiving.
void app_main(void){

    event_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));

    wifi_init();
    bidirectional_init_esp_now();
    uart_init(); 

    xTaskCreate(queue_sm_task, "queue_task", 2048, NULL, 16, NULL);
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
    memcpy(peer_info->peer_addr, MAC1, ESP_NOW_ETH_ALEN);
#else 
    memcpy(peer_info->peer_addr, MAC2, ESP_NOW_ETH_ALEN);
#endif
    peer_info->channel = CONFIG_ESPNOW_CHANNEL;
    peer_info->encrypt = false;
    /* add peer to peer list */
    ESP_ERROR_CHECK(esp_now_add_peer(peer_info));
    ESP_LOGI(TAG, "Init ESPNOW complete");
    free(peer_info);

    return ESP_OK;
}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len){

    espnow_event_t event;
    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

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

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending data");

    for (;;){
        while (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdTRUE){
            printf("\nIN QUEUE [ESPNOW].\n");
            switch(event.id){
                
                case ESPNOW_RECV_CB:

                    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;
                    ESP_LOGI(TAG, "%d bytes incoming from "MACSTR"", recv_cb->data_len, MAC2STR(recv_cb->mac_addr));
                    espnow_data_parse(recv_cb->mac_addr, recv_cb->data, recv_cb-> data_len);
                    break;

                case ESPNOW_SEND_CB:

                    espnow_event_send_cb_t *send_cb = &event.info.send_cb;
                    ESP_LOGI(TAG, "Sending Data to: "MACSTR"", MAC2STR(send_cb->mac_addr));
                    ESP_LOGI(TAG, "Send status: %d", send_cb->status);
                    break;

                default:

                    ESP_LOGE(TAG, "Callback type error: %d", event.id);
                    break;

            }
        }
    }
}

static void uart_event_task(void *pvParameters){

    //Waiting for UART event.
    uart_event_t uart_event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);

    for (;;) {
        if(xQueueReceive(uart_event_queue, (void * )&uart_event, (TickType_t)portMAX_DELAY)) {

            //is it necessary to zero the buffer?
            bzero(dtmp, UART_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_PORT_NUM);
            switch(uart_event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    /* malloc send param space for packing in prepare_data function*/
                    send_param = malloc(sizeof(espnow_send_param_t));
                    if (send_param == NULL){
                        ESP_LOGE(TAG, "Failed to allocate memory for send parameter.");
                        vTaskDelete(NULL);
                    }

                    ESP_LOGI(TAG, "UART Data: %d", uart_event.size);
                    uart_read_bytes(UART_PORT_NUM, dtmp, uart_event.size, portMAX_DELAY);
                    uart_data_prepare(dtmp, uart_event.size);

                    if(esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK){
                        ESP_LOGE(TAG, "Send error");
                        espnow_shutdown();
                        vTaskDelete(NULL);
                    }

                    break;

                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;

                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;

                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_PORT_NUM,  UART_BUF_SIZE);
                    int pos = uart_pattern_pop_pos(UART_PORT_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, UART_BUF_SIZE);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_PORT_NUM);
                    } else {
                        uart_read_bytes(UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_PORT_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
             
                default:
                    ESP_LOGI(TAG, "uart event type: %d", uart_event.type);
                    break;
                    
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);

}

/* in case of failure, it is our responsibility to free memory to avoid memory leaks.*/
void espnow_shutdown(){
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(uart_event_queue);
    vSemaphoreDelete(event_queue);
    esp_now_deinit();
}

//TODO: FIGURE OUT PACKING AND SENDING
/* prepare and pack send param */
void uart_data_prepare(uint8_t *data, int len){

    if (len!=CONFIG_DATA_LEN){
        ESP_LOGE(TAG, "Invalid command length");
        return;
    }
    
    send_param-> unicast = true;
    send_param-> broadcast = false;
    send_param-> count = CONFIG_ESPNOW_SEND_COUNT;
    send_param-> len = CONFIG_ESPNOW_SEND_LEN;
#if DEVICE_SWITCH
    memcpy(send_param->dest_mac, MAC1, ESP_NOW_ETH_ALEN);
#else 
    memcpy(send_param->dest_mac, MAC2, ESP_NOW_ETH_ALEN);
#endif
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);

    if (send_param -> buffer == NULL){
        ESP_LOGE(TAG, "Malloc send buffer fail.");
        free(send_param);
    }

    espnow_data_t *buf = (espnow_data_t *)send_param-> buffer;
    assert(send_param->len >= sizeof(espnow_data_t));

    // TODO: Ensure correct data is being s
    printf("Preparing data: %i steps in the %i direction", data[0], data[2]);
    buf-> type = ESPNOW_DATA_UNICAST;
    buf -> steps = data[0];
    buf -> dir = data[2];


    for (int i = 0; i< CONFIG_DATA_LEN; i++){
        printf("\nData: %i ", data[i]);

    }
}

esp_err_t uart_init(){

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    /* install function args: S-O RX ring buffer, UART num, S-O TX ring buffer, EVENT Q handle and size, flag and interrupt */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, 20));
    
    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Init UART complete \n");

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    return ESP_OK;
}   


void espnow_data_parse(const uint8_t *mac_addr, const uint8_t *data, int len){

    espnow_data_t *buf = (espnow_data_t *)data;
    uint8_t steps, dir =0;
    if(len<sizeof(espnow_data_t)){
        ESP_LOGE(TAG, "Incorrect data size. %d!=%d", len, CONFIG_DATA_LEN);
        free(data); 
    }
    steps = buf -> steps;
    dir = buf -> dir;
    
    ESP_LOGI(TAG, "Received data from :"MACSTR" STEPS: [%d], DIRECTION: [%d]",MAC2STR(mac_addr), steps, dir);
}
