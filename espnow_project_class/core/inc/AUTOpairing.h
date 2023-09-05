#pragma once

//------- C HEADERS ----------//
#include "stdint.h"
#include <string>
#include <queue>
// #include "string.h"
//------- ESP32 HEADERS .- FREERTOS ----------//
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//------- ESP32 HEADERS .- WIFI & ESPNOW ----------//
#include "esp_system.h"
#include "esp_event.h"
// #include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include "esp_efuse.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
//------- ESP32 HEADERS .- FLASH ----------//
#include "nvs_flash.h"
//------- ESP32 HEADERS .- OTA ----------//
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
// #include "espnow_example.h"
#include "AUTOpairing_common.h"

using namespace std;

#define ESP_MAXIMUM_RETRY 5
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

/*---------------------------------------------------------------
		OTA General Macros
---------------------------------------------------------------*/
#define OTA_URL_SIZE 256
#define HASH_LEN 32
#define OTA_RECV_TIMEOUT 5000
#define HTTP_REQUEST_SIZE 16384

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
/*---------------------------------------------------------------
		WiFi variables and ESP-NOW def.
		- Create event group with "EventGroupHandle_t" to signal when we are connected
		- Initialize max number to try connections
		- Set up esp-now channel and some variables to start de pairing
		- Set up update flag
		- Creat queue and semaphore to coordinate the send procedure.
		- Stablish broadcast mac by default
---------------------------------------------------------------*/

static QueueHandle_t cola_resultado_enviados;
static SemaphoreHandle_t semaforo_envio;
static SemaphoreHandle_t semaforo_listo;
TaskHandle_t conexion_hand = NULL;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

static const char *TAG = "* mainApp";
static const char *TAG2 = "* adc_reads";
static const char *TAG3 = "* task conexion";
static const char *TAG4 = "* funcion envio";
static const char *TAG5 = "* init espnow";
static const char *TAG6 = "* callbacks espnow";
static const char *TAG7 = "* deep sleep";
static const char *TAG8 = "* funcion recivo";
static const char *TAG9 = "* advanced https ota";
static const char *TAG10 = "* wifi station";
static const char *TAG11 = "* nvs init";

#define pdSECOND pdMS_TO_TICKS(1000)

#ifndef MAX_CONFIG_SIZE
#define MAX_CONFIG_SIZE 64
#endif

typedef struct
{
	struct timeval sleep_enter_time;
} struct_rtc;

static RTC_DATA_ATTR struct_rtc rtcData;

class AUTOpairing_t
{
	static AUTOpairing_t *this_object;
	typedef enum
	{
		PAIR_REQUEST,
		PAIR_REQUESTED,
		PAIR_PAIRED,
	} PairingStatus;
	typedef struct
	{
		uint16_t code1;
		uint16_t code2;
		struct_pairing data;
		uint16_t config[MAX_CONFIG_SIZE]; // max config size
	} struct_nvs;

	// Others control vars
	int config_size; // check
	int mensajes_sent;
	PairingStatus pairingStatus;
	struct_nvs nvsData;
	struct_pairing pairingData;
	bool mensaje_enviado; // check
	bool terminar;
	unsigned long start_time;
	unsigned long previousMillis_scanChannel; // will store last time channel was scanned
	bool esperando;
	bool esperando_pan;
	bool rtc_init;

	void (*user_callback_mqtt)(struct_espnow_rcv_msg *);
	void (*user_callback_pan)(struct_espnow_rcv_msg *);
	// User control vars
	bool debug;
	unsigned long timeOut;
	bool timeOutEnabled;
	bool nvs_start;
	uint8_t espnow_channel;
	int wakeup_time_sec;
	int panAddress;
	const char *fw_upgrade_url;
	const char *wifi_ssid;
	const char *wifi_pass;

public:
	AUTOpairing_t()
	{
		this_object = this;
		pairingStatus = PAIR_REQUEST;
		mensaje_enviado = false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
		terminar = false;		 // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
		esperando = false;
		esperando_pan = false;
		timeOutEnabled = true;
		nvs_start = false;
		rtc_init = false;
		wakeup_time_sec = 10; // tiempo dormido en segundos
		panAddress = 1;
		config_size = MAX_CONFIG_SIZE;
		start_time = 0; // para controlar el tiempo de escaneo
		previousMillis_scanChannel = 0;
		timeOut = 3000;
		debug = true;
		espnow_channel = 6; // canal para empezar a escanear
		mensajes_sent = 0;
		fw_upgrade_url = NULL;
		wifi_ssid = NULL;
		wifi_pass = NULL;
	}

	void esp_set_https_update(const string &url, const string &ssid, const string &pass)
	{
		fw_upgrade_url = strdup(url.c_str());
		wifi_ssid = strdup(ssid.c_str());
		wifi_pass = strdup(pass.c_str());
	}

	void try_update(UpdateStatus updateStatus = THERE_IS_AN_UPDATE_AVAILABLE)
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		switch (updateStatus)
		{
		case THERE_IS_AN_UPDATE_AVAILABLE:
			// ESP_ERROR_CHECK(esp_wifi_stop());
			ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
			wifi_init_sta();
			/*
			 * Ensure to disable any WiFi power save mode, this allows best throughput
			 * and hence timings for overall OTA operation.
			 */
			esp_wifi_set_ps(WIFI_PS_NONE);
			advance_ota_task();
			break;
		case NO_UPDATE_FOUND:
			// get deep sleep enter time
			//						gotoSleep();
			break;
		}
	}

	void init_update()
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		// clean esp now
		ESP_ERROR_CHECK(esp_now_deinit());
		ESP_ERROR_CHECK(esp_wifi_stop());
		ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
		wifi_init_sta();
		/*
		 * Ensure to disable any WiFi power save mode, this allows best throughput
		 * and hence timings for overall OTA operation.
		 */
		esp_wifi_set_ps(WIFI_PS_NONE);
		advance_ota_task();
	}

	static void wifi_event_handler(void *arg, esp_event_base_t event_base,
								   int32_t event_id, void *event_data)
	{
		if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
		{
			esp_wifi_connect();
		}
		else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
		{
			if (s_retry_num < ESP_MAXIMUM_RETRY)
			{
				esp_wifi_connect();
				s_retry_num++;
				ESP_LOGI("* advanced https ota", "retry to connect to the AP");
			}
			else
			{
				xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			}
			ESP_LOGI("* advanced https ota", "connect to the AP fail");
		}
		else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
		{
			ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
			ESP_LOGI("* advanced https ota", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
			s_retry_num = 0;
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		}
	}
	/* Event handler for catching system events */

	static void ota_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
	{
		if (event_base == ESP_HTTPS_OTA_EVENT)
		{
			switch (event_id)
			{
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
		// ESP_HTTPS_OTA_EVENT
		ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &AUTOpairing_t::ota_event_handler, NULL));
		esp_netif_create_default_wifi_sta();

		wifi_init_config_t cfg = esp_wifi_init_config_default();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));

		esp_event_handler_instance_t instance_any_id;
		esp_event_handler_instance_t instance_got_ip;
		ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &AUTOpairing_t::wifi_event_handler, NULL, &instance_any_id));
		ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &AUTOpairing_t::wifi_event_handler, NULL, &instance_got_ip));
		wifi_config_t wifi_config = {};
		strcpy((char *)wifi_config.sta.ssid, wifi_ssid);
		strcpy((char *)wifi_config.sta.password, wifi_pass);
		wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
		// wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;
		/* wifi_config.sta.pmf_cfg = {};
		wifi_config.sta.pmf_cfg.capable = true;
		wifi_config.sta.pmf_cfg.required = false; */
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_start());

		ESP_LOGI("* advanced https ota", "wifi_init_sta finished.");

		/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
		 * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above) */
		EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
											   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
											   pdFALSE,
											   pdFALSE,
											   portMAX_DELAY);
		ESP_LOGI("* advanced https ota", "before eventBits");
		/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
		 * happened. */
		if (bits & WIFI_CONNECTED_BIT)
		{
			ESP_LOGI("* advanced https ota", "connected to ap SSID:%s password:%s",
					 wifi_ssid, wifi_pass);
		}
		else if (bits & WIFI_FAIL_BIT)
		{
			ESP_LOGI("* advanced https ota", "Failed to connect to SSID:%s, password:%s",
					 wifi_ssid, wifi_pass);
		}
		else
		{
			ESP_LOGE("* advanced https ota", "UNEXPECTED EVENT");
		}
	}

	static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
	{
		if (new_app_info == NULL)
		{
			return ESP_ERR_INVALID_ARG;
		}

		const esp_partition_t *running = esp_ota_get_running_partition();
		esp_app_desc_t running_app_info;
		if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
		{
			ESP_LOGI("* advanced https ota", "Running firmware version: %s", running_app_info.version);
			vTaskDelay(20 / portTICK_PERIOD_MS);
		}

		if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0)
		{
			ESP_LOGW("* advanced https ota", "Current running version is the same as a new. We will not continue the update.");
			return ESP_FAIL;
		}

		/**
		 * Secure version check from firmware image header prevents subsequent download and flash write of
		 * entire firmware image. However this is optional because it is also taken care in API
		 * esp_https_ota_finish at the end of OTA update procedure.
		 */
		const uint32_t hw_sec_version = esp_efuse_read_secure_version();
		if (new_app_info->secure_version < hw_sec_version)
		{
			ESP_LOGW("* advanced https ota", "New firmware security version is less than eFuse programmed, %" PRIu32 " < %" PRIu32, new_app_info->secure_version, hw_sec_version);
			return ESP_FAIL;
		}
		return ESP_OK;
	}

	static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
	{
		esp_err_t err = ESP_OK;
		/* Uncomment to add custom headers to HTTP request */
		// err = esp_http_client_set_header(http_client, "Cache-Control", "no-cache");
		return err;
	}

	void advance_ota_task()
	{
		ESP_LOGI("* advanced https ota", "Starting Advanced OTA example");
		vTaskDelay(20 / portTICK_PERIOD_MS);
		esp_err_t ota_finish_err = ESP_OK;
		esp_http_client_config_t config = {};
		config.url = fw_upgrade_url;
		config.timeout_ms = OTA_RECV_TIMEOUT;
		config.keep_alive_enable = true;

		esp_https_ota_config_t ota_config = {};
		ota_config.http_config = &config;
		ota_config.http_client_init_cb = _http_client_init_cb;

		esp_https_ota_handle_t https_ota_handle = NULL;
		esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
		if (err != ESP_OK)
		{
			ESP_LOGE("* advanced https ota", "ESP HTTPS OTA Begin failed");
			esp_restart();
		}

		esp_app_desc_t app_desc;
		err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
		if (err != ESP_OK)
		{
			ESP_LOGE("* advanced https ota", "esp_https_ota_read_img_desc failed");
			goto ota_end;
		}
		err = validate_image_header(&app_desc);
		if (err != ESP_OK)
		{
			ESP_LOGE("* advanced https ota", "image header verification failed");
			goto ota_end;
		}

		while (1)
		{
			err = esp_https_ota_perform(https_ota_handle);
			if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
			{
				break;
			}
			// esp_https_ota_perform returns after every read operation which gives user the ability to
			// monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
			// data read so far.
			ESP_LOGD("* advanced https ota", "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
		}

		if (esp_https_ota_is_complete_data_received(https_ota_handle) != true)
		{
			// the OTA image was not completely received and user can customise the response to this situation.
			ESP_LOGE("* advanced https ota", "Complete data was not received.");
			goto ota_end;
		}
		else
		{
			ota_finish_err = esp_https_ota_finish(https_ota_handle);
			if ((err == ESP_OK) && (ota_finish_err == ESP_OK))
			{
				ESP_LOGI("* advanced https ota", "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				esp_restart();
			}
			else
			{
				if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
				{
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
	//-----------------------------------------------------------
	void set_pan(uint8_t _pan = 1) { panAddress = _pan; }
	//-----------------------------------------------------------
	int get_pan() { return panAddress; }
	//-----------------------------------------------------------
	void set_timeOut(unsigned long _timeOut = 3000, bool _enable = true)
	{
		timeOut = _timeOut;
		timeOutEnabled = _enable;
	}
	//-----------------------------------------------------------

	void set_channel(uint8_t _channel = 6) { espnow_channel = _channel; }
	//-----------------------------------------------------------
	void set_FLASH(bool _nvs_start = false) { nvs_start = _nvs_start; }
	//-----------------------------------------------------------
	void set_debug(bool _debug = true) { debug = _debug; }
	//-----------------------------------------------------------
	void set_deepSleep(int _wakeup_time_sec = 10) { wakeup_time_sec = _wakeup_time_sec; }
	//-----------------------------------------------------------
	bool init_config_size(uint8_t size)
	{
		config_size = size;
		// Initialize NVS
		esp_err_t ret = nvs_flash_init();
		if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
		{
			// 1.OTA app partition table has a smaller NVS partition size than the non-OTA
			// partition table. This size mismatch may cause NVS initialization to fail.
			// 2.NVS partition contains data in new format and cannot be recognized by this version of code.
			// If this happens, we erase NVS partition and initialize NVS again.
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK(ret);
		nvs_handle_t nvs_handle;
		ret = nvs_open("storage", NVS_READONLY, &nvs_handle);
		if (ret != ESP_OK)
		{
			if (debug)
				ESP_LOGI(TAG11, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
		}
		else
		{
			if (debug)
				ESP_LOGI(TAG11, "Open NVS done");
			size_t required_size;
			ret = nvs_get_blob(nvs_handle, "nvs_struct", NULL, &required_size);
			ret = nvs_get_blob(nvs_handle, "nvs_struct", (void *)&nvsData, &required_size);
			switch (ret)
			{
			case ESP_OK:
				if (debug)
					ESP_LOGI(TAG11, "Done");

				if (nvsData.code1 == MAGIC_CODE1 && nvsData.data.channel == 0)
					nvsData.code1 = INVALID_CODE;
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				if (debug)
					ESP_LOGI(TAG11, "The value is not initialized yet!");
				required_size = sizeof(struct_nvs);
				nvsData.config[1] = timeOut;
				nvsData.config[2] = wakeup_time_sec;
				memset(&(nvsData.data), 0, sizeof(struct_pairing));
				break;
			default:
				if (debug)
					ESP_LOGI(TAG11, "Error (%s) reading!\n", esp_err_to_name(ret));
			}
		}
		if (size > MAX_CONFIG_SIZE)
		{
			ESP_LOGI(TAG11, "Espacio reservado en FLASH demasiado pequeño: %d\n Por favor incremente el valor MAX_CONFIG_SIZE", MAX_CONFIG_SIZE);
			return false;
		}
		else
			return true;
	}
	//-----------------------------------------------------------
	bool get_config(uint16_t *config)
	{
		// init check FLASH

		if (nvsData.code2 == MAGIC_CODE2)
		{
			memcpy(config, &(nvsData.config), config_size);
			if (debug)
				ESP_LOGI(TAG11, "Configuración leída de FLASH");
			return true;
		}
		else
		{
			if (debug)
				ESP_LOGI(TAG11, "Sin configuración en FLASH");
			return false;
		}
	}
	//-----------------------------------------------------------
	void set_config(uint16_t *config)
	{
		nvsData.code2 = MAGIC_CODE2;
		memcpy(&(nvsData.config), config, config_size);
		nvs_saving_task();
	}
	//-----------------------------------------------------------
	void nvs_saving_task()
	{
		// Save in NVS
		esp_err_t ret;
		nvs_handle_t nvs_handle;
		ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
		if (ret != ESP_OK)
		{
			if (debug)
				ESP_LOGI(TAG11, "Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
		}
		else
		{
			if (debug)
				ESP_LOGI(TAG11, "Open NVS done\n");
			if (debug)
				ESP_LOGI(TAG11, "Adding text to NVS Struct... ");
			ret = nvs_set_blob(nvs_handle, "nvs_struct", (const void *)&nvsData, sizeof(struct_nvs));
			if (debug)
				ESP_LOGI(TAG11, "writing nvs status: %s", (ret != ESP_OK) ? "Failed!" : "Done");
			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			if (debug)
				ESP_LOGI(TAG11, "Committing updates in NVS ... ");
			ret = nvs_commit(nvs_handle);
			if (debug)
				ESP_LOGI(TAG11, "commit nvs status: %s", (ret != ESP_OK) ? "Failed!" : "Done");
			// Close
			nvs_close(nvs_handle);
		}
	}
	//----------------------------------------------------------
	wifi_init_config_t esp_wifi_init_config_default()
	{
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
	//---------------------------------------------------
	/* WiFi should start before using ESPNOW */
	void wifi_espnow_init(void)
	{
		ESP_ERROR_CHECK(esp_netif_init());
		ESP_ERROR_CHECK(esp_event_loop_create_default());
		wifi_init_config_t cfg = esp_wifi_init_config_default();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
		ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
		ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
		ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
	}
	//----------------------------------------------------
	void begin(void)
	{

		if (debug)
			ESP_LOGI(TAG, "Comienza AUTOpairing...");
		previousMillis_scanChannel = 0;
		start_time = esp_timer_get_time() / 1000;
		semaforo_envio = xSemaphoreCreateBinary();
		semaforo_listo = xSemaphoreCreateBinary();
		cola_resultado_enviados = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_send_cb_t));

		xSemaphoreGive(semaforo_listo);

	ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

	struct timeval now;
	gettimeofday(&now, NULL);
	int sleep_time_ms = (now.tv_sec - rtcData.sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - rtcData.sleep_enter_time.tv_usec) / 1000;

	if(debug) ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
		if (nvsData.code1 == MAGIC_CODE1)
		{
			// recover information saved in NVS memory
			memcpy(&pairingData, &(nvsData.data), sizeof(pairingData));
			// set WiFi channel
			wifi_init_config_t cfg = esp_wifi_init_config_default();
			ESP_ERROR_CHECK(esp_wifi_init(&cfg));
			if (debug)
				ESP_LOGI(TAG5, "Wifi initialized without problems...\n");
			ESP_ERROR_CHECK(esp_wifi_start());
			if (debug)
				ESP_LOGI(TAG5, "Wifi started without problems...\n");
			ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
			if (debug)
				ESP_LOGI(TAG5, "Wifi set mode STA\n");
			ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
			ESP_ERROR_CHECK(esp_wifi_set_channel(pairingData.channel, WIFI_SECOND_CHAN_NONE));
			if (debug)
				ESP_LOGI(TAG5, "Wifi setting channel  = %d\n", pairingData.channel);
			ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
			if (debug)
				ESP_LOGI(TAG5, "Wifi setting promiscuous = false\n");
			ESP_ERROR_CHECK(esp_wifi_disconnect());
			if (debug)
				ESP_LOGI(TAG5, "Wifi  disconnected\n");
			ESP_ERROR_CHECK(esp_now_init());
			if (debug)
				ESP_LOGI(TAG5, "Wifi esp_now initialized\n");

			ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
			ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
			ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif

			if (debug)
				ESP_LOGI(TAG, "Emparejamiento recuperado de la memoria NVS del usuario ");
			if (debug)
				ESP_LOGI(TAG, "%02X:%02X:%02X:%02X:%02X:%02X", pairingData.macAddr[5], pairingData.macAddr[4], pairingData.macAddr[3], pairingData.macAddr[2], pairingData.macAddr[1], pairingData.macAddr[0]);
			if (debug)
				ESP_LOGI(TAG, "en el canal %d en %f ms", pairingData.channel, (float)(esp_timer_get_time() - start_time) / 1000);

			esp_now_peer_info_t peer;
			peer.channel = pairingData.channel;
			peer.ifidx = ESPNOW_WIFI_IF;
			peer.encrypt = false;
			memcpy(peer.peer_addr, pairingData.macAddr, ESP_NOW_ETH_ALEN);
			ESP_ERROR_CHECK(esp_now_add_peer(&peer));
			if (debug)
				ESP_LOGI(TAG8, "ADD PASARELA PEER");
			pairingStatus = PAIR_PAIRED;
			if (debug)
				ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: EMPAREJAMIENTO");
			xSemaphoreGive(semaforo_envio);
			vTaskDelay(40);
		}
		else
		{
			start_connection_task();
		}
	}

	//-------------------------------------------------------------------------
	static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t sendStatus)
	{
		this_object->_espnow_send_cb(mac_addr, sendStatus);
	}
	/* ESPNOW sending or receiving callback function is called in WiFi task.
	 * Users should not do lengthy operations from this task. Instead, post
	 * necessary data to a queue and handle it from a lower priority task. */
	void _espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
	{

		espnow_send_cb_t resultado;
		memcpy(resultado.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
		resultado.status = status;
		if (mac_addr == NULL)
		{
			ESP_LOGE(TAG4, "Send cb arg error");
			return;
		}

		if (debug)
			ESP_LOGI(TAG4, "ENVIO ESPNOW status: %s", (status) ? "ERROR" : "OK");
		if (debug)
			ESP_LOGI(TAG4, "MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
		if (status == 0)
		{
			if (debug)
				ESP_LOGI(TAG4, " >> Exito de entrega");
			if (pairingStatus == PAIR_PAIRED && mensaje_enviado) // será un mensaje a la pasarela, se podría comprobar la mac
			{
				mensaje_enviado = false;
				if (terminar && !esperando && !esperando_pan)
					gotoSleep();
			}
		}
		else
		{
			if (debug)
				ESP_LOGI(TAG4, " >> Error de entrega");
			if (pairingStatus == PAIR_PAIRED && mensaje_enviado)
			{
				// no hemos conseguido hablar con la pasarela emparejada...
				//  invalidamos config en flash;
				memset(&nvsData, 0, sizeof(struct_nvs));
				nvs_saving_task();
				if (debug)
					ESP_LOGI(TAG4, " INFO de emparejamiento invalidada");
				pairingStatus = PAIR_REQUEST; // volvemos a intentarlo?
				mensaje_enviado = false;
				terminar = false;
				gotoSleep();
				//				//				ESP_ERROR_CHECK(esp_wifi_stop());
				//				//				vTaskDelay(100 / portTICK_PERIOD_MS);
				//				//				autopairing_init();
				//				//				vTaskDelay(timeOut / portTICK_PERIOD_MS);
				//				//				gotoSleep();
			}
		}
		if (xQueueSend(cola_resultado_enviados, &resultado, ESPNOW_MAXDELAY) != pdTRUE)
			ESP_LOGW(TAG, "Send send queue fail");
	}
	//-----------------------------------------------------------------------------
	static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
	{
		this_object->_espnow_recv_cb(recv_info, data, len);
	}
	void _espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
	{
		uint8_t *mac_addr = recv_info->src_addr;
		uint8_t type = data[0];
		uint8_t i;
		uint8_t macB[6];
		uint32_t old;
		struct_pairing *punt = (struct_pairing *)data;
		struct_espnow_rcv_msg *my_msg = (struct_espnow_rcv_msg *)malloc(sizeof(struct_espnow_rcv_msg));

		if (mac_addr == NULL || data == NULL || len <= 0)
		{
			ESP_LOGE(TAG4, "Receive cb arg error");
			return;
		}

		if (debug)
			ESP_LOGI(TAG8, "RECEPCION ESPNOW len: %d", len);

		switch (type & MASK_MSG_TYPE)
		{
		case NODATA:
			if ((((type & MASK_PAN) >> PAN_OFFSET)) == 0)
			{
				esperando = false;
				if (debug)
					ESP_LOGI(TAG8, "NO HAY MENSAJES MQTT");
			}
			else
			{
				esperando_pan = false;
				if (debug)
					ESP_LOGI(TAG8, "NO HAY MENSAJES PAN");
			}

			if (debug)
				ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: NODATA");
			xSemaphoreGive(semaforo_envio);

			if (terminar && !esperando && !esperando_pan) gotoSleep();

			break;
		case PAN_DATA:
			if (debug)
				ESP_LOGI(TAG8, "Mensaje recibido PAN");
			/*
			Tratamiento de la información fuera de la rutina de mensajes recividos.
			Utilizar colas.
			*/
			my_msg->payload = (char *)malloc(len - PAN_payload_offset + 1);
			snprintf(my_msg->payload, len - PAN_payload_offset, "%s", (char *)data + PAN_payload_offset);
			memcpy(my_msg->macAddr, data + PAN_MAC_offset, PAN_MAC_size);
			memcpy((void *)&my_msg->ms_old, data + PAN_MSold_offset, PAN_MSold_size);
			if (debug)
				ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: PAN_DATA");
			xSemaphoreGive(semaforo_envio);
			user_callback_pan(my_msg);
			break;
		case DATA:
			if (debug)
				ESP_LOGI(TAG8, "Mensaje recibido MQTT");
			for (i = 0; i < len; i++)
				if (data[i] == '|')
					break;
			my_msg->topic = (char *)malloc(i - 1);
			my_msg->payload = (char *)malloc(len - i - 1);
			snprintf(my_msg->topic, i, "%s", (char *)data + 1);
			snprintf(my_msg->payload, len - i, "%s", (char *)data + i + 1);
			if (debug)
				ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: DATA");
			xSemaphoreGive(semaforo_envio);
			user_callback_mqtt(my_msg);
			//			mqtt_process_msg(my_msg);
			break;
		case PAIRING:
			if (debug)
				ESP_LOGI(TAG8, "canal: %d", punt->channel);
			if (debug)
				ESP_LOGI(TAG8, "MAC: %02X:%02X:%02X:%02X:%02X:%02X", punt->macAddr[5], punt->macAddr[4], punt->macAddr[3], punt->macAddr[2], punt->macAddr[1], punt->macAddr[0]);
			esp_now_peer_info_t peer;
			peer.channel = punt->channel;
			peer.ifidx = ESPNOW_WIFI_IF;
			peer.encrypt = false;
			
			memcpy(peer.peer_addr, punt->macAddr, ESP_NOW_ETH_ALEN);
			ESP_ERROR_CHECK(esp_now_add_peer(&peer));
			memcpy(pairingData.macAddr, punt->macAddr, 6);
			pairingData.channel = punt->channel;
			if (debug)
				ESP_LOGI(TAG8, "ADD PASARELA PEER");
			pairingStatus = PAIR_PAIRED;
			nvsData.code1 = MAGIC_CODE1;
			memcpy(&(nvsData.data), &pairingData, sizeof(pairingData));
			nvs_saving_task();
			if (debug)
				ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: EMPAREJAMIENTO");
			xSemaphoreGive(semaforo_envio);
			break;
		}
	}

	//-----------------------------------------------------------
	void set_mqtt_msg_callback(void (*_user_callback)(struct_espnow_rcv_msg *))
	{
		user_callback_mqtt = _user_callback;
	}
	//-----------------------------------------------------------
	void set_pan_msg_callback(void (*_user_callback)(struct_espnow_rcv_msg *))
	{
		user_callback_pan = _user_callback;
	}
	//------------------------------------------------------------------------
	void check_messages()
	{
		mensaje_enviado = true;
		esperando = true;
		timeOut += 500;
		uint8_t mensaje_esp;
		mensaje_esp = CHECK;
		if (debug)
			ESP_LOGI(TAG8, "Enviando petición de comprobación de mensajes... ");
		esp_now_send(pairingData.macAddr, (uint8_t *)&mensaje_esp, 1);
	}

	bool espnow_send_check(char *mensaje, bool fin = true, uint8_t _msgType = DATA)
	{
		esperando = true;
		esperando_pan = true;
		timeOut += 500;
		return espnow_send(mensaje, fin, _msgType | CHECK);
	}

	//-----------------------------------------------------------
	uint8_t espnow_send(char *mensaje, bool fin = true, uint8_t _msgType = DATA)
	{
		if (debug)
			ESP_LOGI(TAG3, "ESPERANDO SEMAFORO ENVIO");

		if (xSemaphoreTake(semaforo_envio, 3000 / portTICK_PERIOD_MS) == pdFALSE)
		{
			ESP_LOGE(TAG3, "Error esperando a enviar");
			gotoSleep();
			return ERROR_NOT_PAIRED;
			// a dormir?
		}

		_msgType = _msgType | ((panAddress << PAN_OFFSET) & MASK_PAN);
		char *msg = messType2String(_msgType);
		if (debug)
			ESP_LOGI(TAG3, "Sending message type = %lld %s", dec2bin(_msgType), msg);
		free(msg);

		mensaje_enviado = true;
		terminar = fin;
		mensajes_sent++;

		int size = strlen(mensaje);
		if (size > 249)
		{
			ESP_LOGE(TAG3, "Error longitud del mensaje demasido grande: %d\n", size);
			return ERROR_MSG_TOO_LARGE;
		}
		struct struct_espnow mensaje_esp;
		mensaje_esp.msgType = _msgType;
		memcpy(mensaje_esp.payload, mensaje, size);
		if (debug)
			ESP_LOGI(TAG3, "Longitud del mensaje: %d\n", size);
		if (debug)
			ESP_LOGI(TAG3, "mensaje: %s\n", mensaje);
		esp_now_send(pairingData.macAddr, (uint8_t *)&mensaje_esp, size + 1);
		espnow_send_cb_t resultado;
		if (debug)
			ESP_LOGI(TAG3, "ESPERANDO RESULTADO ENVIO");
		if (xQueueReceive(cola_resultado_enviados, &resultado, 200 / portTICK_PERIOD_MS) == pdFALSE)
		{
			ESP_LOGE(TAG3, "Error esperando resultado envío");
			return ERROR_SIN_RESPUESTA;
		}
		// chequar resultado y ver que hacemos
		// también habría que limitar la espera a 100ms por ejemplo
		xSemaphoreGive(semaforo_envio);
		if (debug)
			ESP_LOGI(TAG3, "LIBERADO SEMAFORO ENVIO: FIN ENVIO");
		if (debug)
			ESP_LOGI(TAG3, "ESPNOW_STATUS = %d", resultado.status);
		if (resultado.status)
			return ERROR_ENVIO_ESPNOW;
		else
			return ENVIO_OK;
	}
	//-----------------------------------------------------------
	int mensajes_enviados() { return mensajes_sent; }

	//-----------------------------------------------------------
	bool emparejado() { return (pairingStatus == PAIR_PAIRED); }

	//-----------------------------------------------------------
	bool envio_disponible()
	{
		while (1)
		{
			if (xSemaphoreTake(semaforo_listo, 3000 / portTICK_PERIOD_MS))
			{
				ESP_LOGI(TAG, "LIBERANDO SEMAFORO CONEXION");
				return true;
			}
		}
		// return (pairingStatus == PAIR_PAIRED && mensaje_enviado == false && terminar == false);
	}

	//--------------------------------------------------------
	void gotoSleep()
	{
		// get deep sleep enter time
		gettimeofday(&rtcData.sleep_enter_time, NULL);
		// add some randomness to avoid collisions with multiple devices
		if (debug)
			ESP_LOGI(TAG7, "Apaga y vamonos");
		// enter deep sleep
		esp_deep_sleep_start();
	}
	//---------------------------------------------------------
	esp_err_t setup(void);
	//-----------------------------------------------------------

	void start_connection_task()
	{
		wifi_espnow_init();
		xTaskCreate(&startTaskImpl, "TASK", 4096, this_object, 1, &conexion_hand);
	}

private:
	static void startTaskImpl(void *pvParameters)
	{
		reinterpret_cast<AUTOpairing_t *>(pvParameters)->keep_connection_task();
	}

	void keep_connection_task()
	{
		while (1)
		{
			if (debug)
				ESP_LOGI(TAG2, "Elapsed time = %f", (float)(esp_timer_get_time() - start_time) / 1000);
			if ((esp_timer_get_time() - start_time) / 1000 > timeOut && timeOutEnabled)
			{
				if (debug)
					ESP_LOGI(TAG2, "SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
				if (debug)
					ESP_LOGI(TAG2, "millis = %lld limite: %ld", esp_timer_get_time(), timeOut);
				gotoSleep();
			}
			switch (pairingStatus)
			{
			case PAIR_REQUEST:
			{
				if (debug)
					ESP_LOGI(TAG2, "Pairing request on channel %d", espnow_channel);

				if (debug)
					ESP_LOGI(TAG, "Pairing request on channel %u\n", espnow_channel);
				// clean esp now
				ESP_ERROR_CHECK(esp_now_deinit());
				// set WiFi channel
				uint8_t primary = -1;
				wifi_second_chan_t secondary;
				wifi_init_config_t cfg = esp_wifi_init_config_default();
				ESP_ERROR_CHECK(esp_wifi_init(&cfg));
				if (debug)
					ESP_LOGI(TAG5, "Wifi initialized without problems...\n");
				ESP_ERROR_CHECK(esp_wifi_start());
				if (debug)
					ESP_LOGI(TAG5, "Wifi started without problems...\n");
				ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
				if (debug)
					ESP_LOGI(TAG5, "Wifi set mode STA\n");
				ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
				if (debug)
					ESP_LOGI(TAG5, "Wifi setting promiscuous = true\n");
				ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &secondary));
				if (debug)
					ESP_LOGI(TAG5, "Retrieved channel before setting channel: %d\n", primary);
				ESP_ERROR_CHECK(esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE));
				if (debug)
					ESP_LOGI(TAG5, "Wifi setting channel  = %d\n", espnow_channel);
				ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
				if (debug)
					ESP_LOGI(TAG5, "Wifi setting promiscuous = false\n");
				ESP_ERROR_CHECK(esp_wifi_disconnect());
				if (debug)
					ESP_LOGI(TAG5, "Wifi  disconnected\n");
				ESP_ERROR_CHECK(esp_now_init());
				if (debug)
					ESP_LOGI(TAG5, "Wifi esp_now initialized\n");
				ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &secondary));
				if (debug)
					ESP_LOGI(TAG5, "Retrieved channel after setting it : %d\n", primary);

				ESP_ERROR_CHECK(esp_now_init());
				ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
				ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
				ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif
				/* Set primary master key. */
				/* Add broadcast peer information to peer list. */
				esp_now_peer_info_t peer;
				peer.channel = espnow_channel;
				peer.ifidx = ESPNOW_WIFI_IF;
				peer.encrypt = false;
				memcpy(peer.peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
				ESP_ERROR_CHECK(esp_now_add_peer(&peer));

				// set pairing data to send to the server
				pairingData.msgType = PAIRING; //| ((panAddress << PAN_OFFSET) & MASK_PAN);
				pairingData.id = ESPNOW_DEVICE;

				// send request
				esp_now_send(s_example_broadcast_mac, (uint8_t *)&pairingData, sizeof(pairingData));
				pairingStatus = PAIR_REQUESTED;
				break;
			}

			case PAIR_REQUESTED:
				// time out to allow receiving response from server
				vTaskDelay(100 / portTICK_PERIOD_MS);
				if (pairingStatus == PAIR_REQUESTED)
				// time out expired,  try next channel
				{
					espnow_channel++;
					if (espnow_channel > 11)
						espnow_channel = 1;
					pairingStatus = PAIR_REQUEST;
				}
				break;

			case PAIR_PAIRED:
				vTaskSuspend(conexion_hand);
				break;
			default:
				break;
			}
		}
	}

	//	void keep_connection(void)
	//	{
	//		xTaskCreate(keep_connection_task, "conexion", 4096, NULL, 1, &conexion_hand);
	//	}
};
AUTOpairing_t *AUTOpairing_t::this_object = NULL;
