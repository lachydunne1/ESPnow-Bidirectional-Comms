#ifndef ESPNOW_BIDIR_CONFIG
#define ESPNOW_BIDIR_CONFIG

/* Include destination MAC. 
   The default address is the broadcast address, which works, but sender assumes every tx assumes if this is the case.
   Setting receiver address will allow the sender to determine if sending has succeeded or failed.
*/

/* arbitrarily set. channel must be consistent for sender and receiver*/
#define CONFIG_DEEP_SLEEP_TIME_MS 10000
#define CONFIG_ESPNOW_SEND_COUNT 100
#define CONFIG_ESPNOW_SEND_LEN 5 //questionable
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_DELAY_TIME 10
#define CONFIG_DATA_LEN 3
#define ESPNOW_WIFI_MODE WIFI_MODE_STA

#define ESPNOW_QUEUE_SIZE 5


static uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_RECV_CB,
    ESPNOW_SEND_CB,
}espnow_event_id_t;

typedef struct {
    uint8_t type; /* BOOL Broadcast [1] or UNICAST [0]*/
    uint8_t steps;
    uint8_t dir;
}__attribute__((packed)) espnow_data_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
}espnow_event_recv_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
}espnow_event_send_cb_t;

/* for event handling in queue SM*/
typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
}espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

/* send only structs */
enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
};

typedef struct {
    bool unicast; /* initially get unicast for POC */
    bool broadcast; /* for future use */
    uint8_t count; /* delay count */
    int len; /* data length */
    uint8_t *buffer; /* buffer pointing to the data */
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; /* length of ESPNOW Mac address */
} espnow_send_param_t;


void wifi_init(void);

#endif 