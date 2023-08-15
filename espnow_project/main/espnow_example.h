/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
/*
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif
*/

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA


/*---------------------------------------------------------------
        ESP-NOW & WiFi General Macros
---------------------------------------------------------------*/

#define BOARD_ID 2  // tiene que ser != 0, que sería la pasarela
#define ESPNOW_QUEUE_SIZE           10
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

#define DATA    0b00000001
#define PAIRING 0b00000000
#define SUBS    0b00000010
#define CHECK   0b10000000
#define NODATA  0b00000011

#define CANAL 6

#define ESPNOW_MAXDELAY 512

#define ERROR_NOT_PAIRED    1
#define ERROR_MSG_TOO_LARGE 2
#define ERROR_SIN_RESPUESTA 3
#define ERROR_ENVIO_ESPNOW  4
#define ENVIO_OK            0




/*---------------------------------------------------------------
        EEPROM General Macros
---------------------------------------------------------------*/
#define MAGIC_CODE1 0xA5A5
#define MAGIC_CODE2 0xC7C7
#define INVALID_CODE 0

#ifndef MAX_CONFIG_SIZE
#define MAX_CONFIG_SIZE 64
#endif

typedef enum
{
    PAIR_REQUEST,
    PAIR_REQUESTED,
    PAIR_PAIRED,
} PairingStatus;

typedef enum
{
    NO_UPDATE_FOUND,
    THERE_IS_AN_UPDATE_AVAILABLE,
} UpdateStatus;

typedef struct{      // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
    uint8_t padding[3];
}struct_pairing;


struct struct_espnow {      // esp-now message structure
    uint8_t msgType;
    uint8_t payload[249];
};


typedef struct{
	uint8_t tsleep;
	uint8_t pan;
	uint16_t timeout;
}struct_config;

typedef struct{
  uint16_t code1;
  uint16_t code2;
  struct_pairing data;
  uint16_t config[MAX_CONFIG_SIZE]; // max config size
}struct_rtc;



typedef struct {
	uint8_t mac_addr[ESP_NOW_ETH_ALEN];
	esp_now_send_status_t status;
} espnow_send_cb_t;



typedef struct{
	char *topic;
	char *payload;
	uint8_t macAddr[6];
}struct_espnow_rcv_msg;




#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#endif
