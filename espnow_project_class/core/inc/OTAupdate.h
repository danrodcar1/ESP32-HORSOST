#pragma once

//------- C HEADERS ----------//
#include "stdint.h"
#include <string>
//------- ESP32 HEADERS .- WIFI & ESPNOW ----------//
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_efuse.h"
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


using namespace std;

#define pdSECOND pdMS_TO_TICKS(1000)

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
#define FIRMWARE_UPGRADE_URL "https://huertociencias.uma.es/ESP32OTA/espnow_project.bin"
#define OTA_RECV_TIMEOUT 5000
#define HTTP_REQUEST_SIZE 16384

EventGroupHandle_t s_wifi_event_group;
int s_retry_num = 0;

class OTAupdate_t
{
	static OTAupdate_t *this_object;


public:
	OTAupdate_t()
	{
		this_object = this;
	}

	//----------------------------------------------------------
	wifi_init_config_t esp_wifi_init_config_default(){
		wifi_init_config_t cfg = {};
		cfg.osi_funcs = &g_wifi_osi_funcs;
		cfg.wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs;
		cfg.static_rx_buf_num = CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM;
		cfg.dynamic_rx_buf_num = CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM;
		cfg.tx_buf_type = CONFIG_ESP32_WIFI_TX_BUFFER_TYPE;
		cfg.static_tx_buf_num = WIFI_STATIC_TX_BUFFER_NUM;
		cfg.dynamic_tx_buf_num = WIFI_DYNAMIC_TX_BUFFER_NUM;
		cfg.cache_tx_buf_num = WIFI_CACHE_TX_BUFFER_NUM;
		cfg.csi_enable = WIFI_CSI_ENABLED;
		cfg.ampdu_rx_enable = WIFI_AMPDU_RX_ENABLED;
		cfg.ampdu_tx_enable = WIFI_AMPDU_TX_ENABLED;
		cfg.amsdu_tx_enable = WIFI_AMSDU_TX_ENABLED;
		cfg.nvs_enable = WIFI_NVS_ENABLED;
		cfg.nano_enable = WIFI_NANO_FORMAT_ENABLED;
		cfg.rx_ba_win = WIFI_DEFAULT_RX_BA_WIN;
		cfg.wifi_task_core_id = WIFI_TASK_CORE_ID;
		cfg.beacon_max_len = WIFI_SOFTAP_BEACON_MAX_LEN;
		cfg.mgmt_sbuf_num = WIFI_MGMT_SBUF_NUM;
		cfg.feature_caps = g_wifi_feature_caps;
		cfg.sta_disconnected_pm = WIFI_STA_DISCONNECTED_PM_ENABLED;
		cfg.espnow_max_encrypt_num = CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM;
		cfg.magic = WIFI_INIT_CONFIG_MAGIC;
		return cfg;
	}

	static void wifi_event_handler(void* arg, esp_event_base_t event_base,
			int32_t event_id, void* event_data)
	{
		if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
			esp_wifi_connect();
		} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
			if (s_retry_num < ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				s_retry_num++;
				ESP_LOGI("* advanced https ota", "retry to connect to the AP");
			} else {
				xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			}
			ESP_LOGI("* advanced https ota","connect to the AP fail");
		} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
			ESP_LOGI("* advanced https ota", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
			s_retry_num = 0;
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		}
	}
	/* Event handler for catching system events */

	static void ota_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
	{
		if (event_base == ESP_HTTPS_OTA_EVENT) {
			switch (event_id) {
			case ESP_HTTPS_OTA_START:
				ESP_LOGI("* advanced https ota", "OTA started");
				break;
			case ESP_HTTPS_OTA_CONNECTED:
				ESP_LOGI("* advanced https ota", "Connected to server");
				break;
			case ESP_HTTPS_OTA_GET_IMG_DESC:
				ESP_LOGI("* advanced https ota", "Reading Image Description");
				break;
			case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
				ESP_LOGI("* advanced https ota", "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
				break;
			case ESP_HTTPS_OTA_DECRYPT_CB:
				ESP_LOGI("* advanced https ota", "Callback to decrypt function");
				break;
			case ESP_HTTPS_OTA_WRITE_FLASH:
				ESP_LOGD("* advanced https ota", "Writing to flash: %d written", *(int *)event_data);
				break;
			case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
				ESP_LOGI("* advanced https ota", "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
				break;
			case ESP_HTTPS_OTA_FINISH:
				ESP_LOGI("* advanced https ota", "OTA finish");
				break;
			case ESP_HTTPS_OTA_ABORT:
				ESP_LOGI("* advanced https ota", "OTA abort");
				break;
			}
		}
	}

	void wifi_init_sta(void)
	{

		s_wifi_event_group = xEventGroupCreate();

		ESP_ERROR_CHECK(esp_netif_init());

		ESP_ERROR_CHECK(esp_event_loop_create_default());
		//ESP_HTTPS_OTA_EVENT
		ESP_ERROR_CHECK(esp_event_handler_register(NULL, ESP_EVENT_ANY_ID, &OTAupdate_t::ota_event_handler, NULL));
		esp_netif_create_default_wifi_sta();

		wifi_init_config_t cfg = esp_wifi_init_config_default();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));

		esp_event_handler_instance_t instance_any_id;
		esp_event_handler_instance_t instance_got_ip;
		ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &OTAupdate_t::wifi_event_handler, NULL, &instance_any_id));
		ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &OTAupdate_t::wifi_event_handler, NULL, &instance_got_ip));
		wifi_config_t wifi_config = {};
		strcpy((char*)wifi_config.sta.ssid, ESP_WIFI_SSID);
		strcpy((char*)wifi_config.sta.password, ESP_WIFI_PASS);
		wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
		wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;

		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
		ESP_ERROR_CHECK(esp_wifi_start() );

		ESP_LOGI("* advanced https ota", "wifi_init_sta finished.");

		/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
		 * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above) */
		EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
				WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
				pdFALSE,
				pdFALSE,
				portMAX_DELAY);

		/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
		 * happened. */
		if (bits & WIFI_CONNECTED_BIT) {
			ESP_LOGI("* advanced https ota", "connected to ap SSID:%s password:%s",
					ESP_WIFI_SSID, ESP_WIFI_PASS);
		} else if (bits & WIFI_FAIL_BIT) {
			ESP_LOGI("* advanced https ota", "Failed to connect to SSID:%s, password:%s",
					ESP_WIFI_SSID, ESP_WIFI_PASS);
		} else {
			ESP_LOGE("* advanced https ota", "UNEXPECTED EVENT");
		}
	}

	static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
	{
		if (new_app_info == NULL) {
			return ESP_ERR_INVALID_ARG;
		}

		const esp_partition_t *running = esp_ota_get_running_partition();
		esp_app_desc_t running_app_info;
		if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
			ESP_LOGI("* advanced https ota", "Running firmware version: %s", running_app_info.version);
			vTaskDelay(20 / portTICK_PERIOD_MS);
		}

		if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
			ESP_LOGW("* advanced https ota", "Current running version is the same as a new. We will not continue the update.");
			return ESP_FAIL;
		}

		/**
		 * Secure version check from firmware image header prevents subsequent download and flash write of
		 * entire firmware image. However this is optional because it is also taken care in API
		 * esp_https_ota_finish at the end of OTA update procedure.
		 */
		const uint32_t hw_sec_version = esp_efuse_read_secure_version();
		if (new_app_info->secure_version < hw_sec_version) {
			ESP_LOGW("* advanced https ota", "New firmware security version is less than eFuse programmed, %"PRIu32" < %"PRIu32, new_app_info->secure_version, hw_sec_version);
			return ESP_FAIL;
		}
		return ESP_OK;
	}


	static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
	{
		esp_err_t err = ESP_OK;
		/* Uncomment to add custom headers to HTTP request */
		//err = esp_http_client_set_header(http_client, "Cache-Control", "no-cache");
		return err;
	}

	void advance_ota_task()
	{
		ESP_LOGI("* advanced https ota", "Starting Advanced OTA example");
		vTaskDelay(20 / portTICK_PERIOD_MS);
		esp_err_t ota_finish_err = ESP_OK;
		esp_http_client_config_t config = {};
		config.url = FIRMWARE_UPGRADE_URL;
		config.timeout_ms = OTA_RECV_TIMEOUT;
		config.keep_alive_enable = true;

		esp_https_ota_config_t ota_config = {};
		ota_config.http_config = &config;
		ota_config.http_client_init_cb = _http_client_init_cb;

		esp_https_ota_handle_t https_ota_handle = NULL;
		esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
		if (err != ESP_OK) {
			ESP_LOGE("* advanced https ota", "ESP HTTPS OTA Begin failed");
			esp_restart();
		}

		esp_app_desc_t app_desc;
		err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
		if (err != ESP_OK) {
			ESP_LOGE("* advanced https ota", "esp_https_ota_read_img_desc failed");
			goto ota_end;
		}
		err = validate_image_header(&app_desc);
		if (err != ESP_OK) {
			ESP_LOGE("* advanced https ota", "image header verification failed");
			goto ota_end;
		}

		while (1) {
			err = esp_https_ota_perform(https_ota_handle);
			if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
				break;
			}
			// esp_https_ota_perform returns after every read operation which gives user the ability to
			// monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
			// data read so far.
			ESP_LOGD("* advanced https ota", "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
		}

		if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
			// the OTA image was not completely received and user can customise the response to this situation.
			ESP_LOGE("* advanced https ota", "Complete data was not received.");
			goto ota_end;
		} else {
			ota_finish_err = esp_https_ota_finish(https_ota_handle);
			if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
				ESP_LOGI("* advanced https ota", "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				esp_restart();
			} else {
				if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
					ESP_LOGE("* advanced https ota", "Image validation failed, image is corrupted");
				}
				ESP_LOGE("* advanced https ota", "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
				goto ota_end;
			}
		}

		ota_end:
		esp_https_ota_abort(https_ota_handle);
		ESP_LOGE("* advanced https ota", "ESP_HTTPS_OTA upgrade failed");
		esp_restart();
	}


};
OTAupdate_t *OTAupdate_t::this_object=NULL;
