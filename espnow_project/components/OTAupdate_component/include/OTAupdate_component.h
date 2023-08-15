#include <stdio.h>
//------- ESP32 HEADERS .- WIFI & ESPNOW ----------//
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_efuse.h"
//------- ESP32 HEADERS .- DEEP SLEEP ----------//
#include "esp_sleep.h"
//------- ESP32 HEADERS .- FREERTOS ----------//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//------- ESP32 HEADERS .- FLASH ----------//
#include "nvs_flash.h"
//------- ESP32 HEADERS .- OTA ----------//
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#define ESP_WIFI_SSID      "infind"
#define ESP_WIFI_PASS      "1518wifi"
#define ESP_MAXIMUM_RETRY  5
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID


#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

/*---------------------------------------------------------------
        OTA General Macros
---------------------------------------------------------------*/
#define OTA_URL_SIZE 256
#define HASH_LEN 32
#define FIRMWARE_UPGRADE_URL "https://huertociencias.uma.es/ESP32OTA/espnow_example.bin"
#define OTA_RECV_TIMEOUT 5000
#define HTTP_REQUEST_SIZE 16384


void func(void);

void wifi_init_sta(void);
void advance_ota_task();
