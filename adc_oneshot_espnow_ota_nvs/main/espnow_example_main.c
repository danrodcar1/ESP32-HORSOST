/* ESPNOW Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

/*
 This example shows how to use ESPNOW.
 Prepare two device, one for sending ESPNOW data and another for receiving
 ESPNOW data.
 */
//------- C HEADERS ----------//
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <string.h>
#include <assert.h>
//------- ESP32 HEADERS .- FREERTOS ----------//
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//------- ESP32 HEADERS .- GPIO ----------//
#include "driver/gpio.h"
//------- ESP32 HEADERS .- FLASH ----------//
#include "nvs.h"
#include "nvs_flash.h"
//------- ESP32 HEADERS .- WIFI & ESPNOW ----------//
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_efuse.h" // CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
#include "espnow_example.h"
#include "AUTOpairing_common.h"
//------- ESP32 HEADERS .- ONESHOT ADC ----------//
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
//------- ESP32 HEADERS .- DEEP SLEEP ----------//
#include "esp_sleep.h"
//------- ESP32 HEADERS .- JSON FORMATTER ----------//
#include "cJSON.h"
//------- ESP32 HEADERS .- OTA ----------//
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "protocol_examples_common.h"


static const char *TAG  = "* mainApp";
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


/*---------------------------------------------------------------
        WiFi variables and ESP-NOW def.
        - Create event group with "EventGroupHandle_t" to signal when we are connected
        - Initialize max number to try connections
        - Set up esp-now channel and some variables to start de pairing
        - Set up update flag
        - Creat queue and semaphore to coordinate the send procedure.
        - Stablish broadcast mac by default
---------------------------------------------------------------*/
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;
static int espnow_channel = CANAL;
PairingStatus pairingStatus=PAIR_REQUEST;
UpdateStatus updateStatus = NO_UPDATE_FOUND;
TaskHandle_t conexion_hand = NULL;
static QueueHandle_t cola_resultado_enviados;
static SemaphoreHandle_t semaforo_envio;

struct_pairing pairingData;
struct_config nvsConfig;
struct_rtc nvsData;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

// Others control vars
int wakeup_time_sec = 10;
int mensajes_sent = 0;
int panAddress = 1;
int config_size = 0;
bool esperando = false;
bool terminar = false;
bool mensaje_enviado = false;
bool debug = false;
bool nvs_enable = true;
uint16_t timeOut = 3000;
unsigned long start_time;
bool timeOutEnabled = false;
/*---------------------------------------------------------------
        ADC variables
        - Create task handle to know when we have to point the ADC conversor
        - Do the same with the calibration unit
        - Set up adc channels
        - Set up convertion-time studied for one analog sensor previously
---------------------------------------------------------------*/

//ADC1 Channels
TaskHandle_t adcTaskHandle = NULL;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;
const int adc_channel[1] = {ADC_CHANNEL_0};
//static adc_channel_t adc_channel[4] = {ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_4};
uint8_t lengthADC1_CHAN = sizeof(adc_channel) / sizeof(adc_channel_t);

float EMA_ALPHA = 0.6;

unsigned long convertion_time = 200;

/*---------------------------------------------------------------
        This struc will storage in slow RTC memory some vars
		- struct timeval cointains Structure returned by gettimeofday(2) system call, and used in other calls
			-> tv_sec = seconds
			-> tv_usec = microseconds
---------------------------------------------------------------*/
typedef struct{
	struct timeval sleep_enter_time;
}struct_rtcdata;

static RTC_DATA_ATTR struct_rtcdata rtcData;

// Function prototypes
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
uint8_t espnow_send(char * mensaje, bool fin, uint8_t _msgType);
void gotoSleep();
void autopairing_init();
static inline void nvs_saving_task();

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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
			ESP_LOGI(TAG9, "OTA started");
			break;
		case ESP_HTTPS_OTA_CONNECTED:
			ESP_LOGI(TAG9, "Connected to server");
			break;
		case ESP_HTTPS_OTA_GET_IMG_DESC:
			ESP_LOGI(TAG9, "Reading Image Description");
			break;
		case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
			ESP_LOGI(TAG9, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
			break;
		case ESP_HTTPS_OTA_DECRYPT_CB:
			ESP_LOGI(TAG9, "Callback to decrypt function");
			break;
		case ESP_HTTPS_OTA_WRITE_FLASH:
			ESP_LOGD(TAG9, "Writing to flash: %d written", *(int *)event_data);
			break;
		case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
			ESP_LOGI(TAG9, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
			break;
		case ESP_HTTPS_OTA_FINISH:
			ESP_LOGI(TAG9, "OTA finish");
			break;
		case ESP_HTTPS_OTA_ABORT:
			ESP_LOGI(TAG9, "OTA abort");
			break;
		}
	}
}

void wifi_init_sta(void)
{

	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, NULL));
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

	wifi_config_t wifi_config = {
			.sta = {
					.ssid = ESP_WIFI_SSID,
					.password = ESP_WIFI_PASS,
					/* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
					 * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
					 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
					 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
					 */
					.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
					.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
			},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");

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
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
				ESP_WIFI_SSID, ESP_WIFI_PASS);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
				ESP_WIFI_SSID, ESP_WIFI_PASS);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
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
		ESP_LOGI(TAG9, "Running firmware version: %s", running_app_info.version);
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}

	if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
		ESP_LOGW(TAG9, "Current running version is the same as a new. We will not continue the update.");
		return ESP_FAIL;
	}
	/**
	 * Secure version check from firmware image header prevents subsequent download and flash write of
	 * entire firmware image. However this is optional because it is also taken care in API
	 * esp_https_ota_finish at the end of OTA update procedure.
	 */
	const uint32_t hw_sec_version = esp_efuse_read_secure_version();
	if (new_app_info->secure_version < hw_sec_version) {
		ESP_LOGW(TAG9, "New firmware security version is less than eFuse programmed, %"PRIu32" < %"PRIu32, new_app_info->secure_version, hw_sec_version);
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
	ESP_LOGI(TAG9, "Starting Advanced OTA example");
	vTaskDelay(20 / portTICK_PERIOD_MS);
	esp_err_t ota_finish_err = ESP_OK;
	esp_http_client_config_t config = {
			.url = FIRMWARE_UPGRADE_URL,
			.timeout_ms = OTA_RECV_TIMEOUT,
			.keep_alive_enable = true,
	};


	esp_https_ota_config_t ota_config = {
			.http_config = &config,
			.http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
			//        .partial_http_download = true,
			//        .max_http_request_size = HTTP_REQUEST_SIZE,
	};

	esp_https_ota_handle_t https_ota_handle = NULL;
	esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG9, "ESP HTTPS OTA Begin failed");
		updateStatus = NO_UPDATE_FOUND;
		gotoSleep();
	}

	esp_app_desc_t app_desc;
	err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
	if (err != ESP_OK) {
		ESP_LOGE(TAG9, "esp_https_ota_read_img_desc failed");
		goto ota_end;
	}
	err = validate_image_header(&app_desc);
	if (err != ESP_OK) {
		ESP_LOGE(TAG9, "image header verification failed");
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
		ESP_LOGD(TAG9, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
	}

	if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
		// the OTA image was not completely received and user can customise the response to this situation.
		ESP_LOGE(TAG9, "Complete data was not received.");
		goto ota_end;
	} else {
		ota_finish_err = esp_https_ota_finish(https_ota_handle);
		if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
			ESP_LOGI(TAG9, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			esp_restart();
		} else {
			if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
				ESP_LOGE(TAG9, "Image validation failed, image is corrupted");
			}
			ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
			goto ota_end;
		}
	}

	ota_end:
	esp_https_ota_abort(https_ota_handle);
	ESP_LOGE(TAG9, "ESP_HTTPS_OTA upgrade failed");
	gotoSleep();
}


//-----------------------------------------------------------
int mensajes_enviados()
{
	return mensajes_sent;
}

//-----------------------------------------------------------
bool emparejado()
{
	return (pairingStatus==PAIR_PAIRED) ;
}

//-----------------------------------------------------------
bool envio_disponible()
{
	return (pairingStatus==PAIR_PAIRED && mensaje_enviado==false && terminar==false) ;
}

//-----------------------------------------------------------
void set_debug(bool _debug)
{
	debug = _debug;
}

//-----------------------------------------------------------
void set_deepSleep(uint16_t _wakeup_time_sec)
{
	wakeup_time_sec=_wakeup_time_sec;
}

void set_timeOut(uint16_t _timeOut, bool _enable)
{
	timeOut = _timeOut;
	timeOutEnabled = _enable;
}

//--------------------------------------------------------
void gotoSleep() {
	// get deep sleep enter time
	gettimeofday(&rtcData.sleep_enter_time, NULL);
	// add some randomness to avoid collisions with multiple devices
	if(debug) ESP_LOGI(TAG7, "Apaga y vamonos");
	// enter deep sleep
	esp_deep_sleep_start();
}


/* WiFi should start before using ESPNOW */
static void wifi_init(void) {
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
	ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
	ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

void check_messages()
{
	mensaje_enviado = true;
	esperando = true;
	timeOut += 500;
	uint8_t mensaje_esp;
	mensaje_esp = CHECK;
	if(debug) ESP_LOGI(TAG8, "Enviando petición de comprobación de mensajes... ");
	esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, 1);
}

bool espnow_send_check(char * mensaje, bool fin, uint8_t _msgType)
{
	esperando = true;
	timeOut += 500;
	return espnow_send(mensaje, fin, _msgType | CHECK);
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr,	esp_now_send_status_t status) {

	espnow_send_cb_t resultado;
	memcpy(resultado.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
	resultado.status = status;
	if (mac_addr == NULL) {
		ESP_LOGE(TAG4, "Send cb arg error");
		return;
	}

	if(debug) ESP_LOGI(TAG4, "ENVIO ESPNOW status: %s", (status)?"ERROR":"OK");
	if(debug) ESP_LOGI(TAG4, "MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[5],mac_addr[4],mac_addr[3],mac_addr[2],mac_addr[1],mac_addr[0]);
//	if(status == 0){
		if(debug) ESP_LOGI(TAG4, " >> Exito de entrega");
		if(pairingStatus==PAIR_PAIRED && mensaje_enviado)  // será un mensaje a la pasarela, se podría comprobar la mac
		{
			mensaje_enviado = false;
			if (terminar && !esperando) gotoSleep();
		}
//	}
//	else{
//		if(debug) ESP_LOGI(TAG4, " >> Error de entrega");
//		if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
//		{
//			//no hemos conseguido hablar con la pasarela emparejada...
//			// invalidamos config en flash;
//			memset(&nvsData, 0, sizeof(struct_rtc));
//			nvs_saving_task();
//			if(debug)  ESP_LOGI(TAG4, " INFO de emparejamiento invalidada");
//			pairingStatus = PAIR_REQUEST; // volvemos a intentarlo?
//			mensaje_enviado=false;
//			terminar=false;
//			ESP_ERROR_CHECK(esp_wifi_stop());
//			vTaskDelay(100 / portTICK_PERIOD_MS);
//			autopairing_init();
//			vTaskDelay(timeOut / portTICK_PERIOD_MS);
//			gotoSleep();
//		}
//	}

	if(xQueueSend(cola_resultado_enviados,&resultado, ESPNOW_MAXDELAY) != pdTRUE)ESP_LOGW(TAG, "Send send queue fail");
}

static inline void nvs_saving_task(){
	// Save in NVS
	esp_err_t ret;
	nvs_handle_t nvs_handle;
	ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
	if (ret != ESP_OK) {
		if(debug) ESP_LOGI(TAG11, "Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
	} else {
		if(debug) ESP_LOGI(TAG11, "Open NVS done\n");
		if(debug) ESP_LOGI(TAG11, "Adding text to NVS Struct... ");
		ret = nvs_set_blob(nvs_handle, "nvs_struct", (const void*)&nvsData, sizeof(struct_rtc));
		if(debug) ESP_LOGI(TAG11, "writing nvs status: %s", (ret != ESP_OK) ? "Failed!" : "Done");
		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		if(debug) ESP_LOGI(TAG11, "Committing updates in NVS ... ");
		ret = nvs_commit(nvs_handle);
		if(debug) ESP_LOGI(TAG11, "commit nvs status: %s", (ret != ESP_OK) ? "Failed!" : "Done");
		// Close
		nvs_close(nvs_handle);
	}
}

static esp_err_t mqtt_process_msg(struct_espnow_rcv_msg *my_msg){
	if(debug) ESP_LOGI(TAG8, "topic: %s", my_msg->topic);
	cJSON *root2 = cJSON_Parse(my_msg->payload);
	if(strcmp (my_msg->topic,"config") == 0){
		if(debug) ESP_LOGI(TAG8, "payload: %s", my_msg->payload);
		if(debug) ESP_LOGI(TAG8, "Deserialize payload.....");
		cJSON *sleep = cJSON_GetObjectItem(root2,"sleep");
		cJSON *timeout = cJSON_GetObjectItem(root2,"timeout");
		cJSON *pan = cJSON_GetObjectItem(root2,"pan");

		if(timeout) nvsData.config[1] = timeout->valueint;
		if(sleep) nvsData.config[2] = sleep->valueint;
		if(pan) nvsData.config[3] = pan->valueint;
		if(debug) ESP_LOGI(TAG8, "tsleep = %d",nvsData.config[1]);
		if(debug) ESP_LOGI(TAG8, "timeout = %d",nvsData.config[2]);
		if(debug) ESP_LOGI(TAG8, "PAN_ID = %d",nvsData.config[3]);
		nvs_saving_task();
	}
	if(strcmp (my_msg->topic,"update") == 0){
		updateStatus = THERE_IS_AN_UPDATE_AVAILABLE;
	}
	free(my_msg->topic);
	free(my_msg->payload);
	cJSON_Delete(root2);
	return ESP_OK;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
	uint8_t * mac_addr = recv_info->src_addr;
	uint8_t type = data[0];
	uint8_t i;
	struct_pairing *punt = (struct_pairing*) data;
	struct_espnow_rcv_msg *my_msg = malloc(sizeof(struct_espnow_rcv_msg));

	if (mac_addr == NULL || data == NULL || len <= 0) {
		ESP_LOGE(TAG4, "Receive cb arg error");
		return;
	}

	if(debug) ESP_LOGI(TAG8, "RECEPCION ESPNOW len: %d", len);

	switch (type & MASK_MSG_TYPE){
	case NODATA:
		esperando = false;
		if(debug) ESP_LOGI(TAG8, "NO HAY MENSAJES MQTT");

		break;
	case DATA:
		if(debug) ESP_LOGI(TAG8, "Mensaje recibido MQTT");
		for(i=0; i<len; i++) if(data[i]=='|') break;
		my_msg->topic = malloc(i - 1);
		my_msg->payload = malloc(len - i - 1);
		snprintf(my_msg->topic, i, "%s", (char*)data + 1);
		snprintf(my_msg->payload, len - i, "%s", (char*)data + i + 1);
		mqtt_process_msg(my_msg);
		break;
	case PAIRING:
		if(debug) ESP_LOGI(TAG8, "canal: %d", punt->channel);
		if(debug) ESP_LOGI(TAG8, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",punt->macAddr[5],punt->macAddr[4],punt->macAddr[3],punt->macAddr[2],punt->macAddr[1],punt->macAddr[0]);
		esp_now_peer_info_t peer;
		peer.channel = punt->channel;
		peer.ifidx = ESPNOW_WIFI_IF;
		peer.encrypt = false;
		memcpy(peer.peer_addr, punt->macAddr, ESP_NOW_ETH_ALEN);
		ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
		memcpy(pairingData.macAddr, punt->macAddr, 6);
		pairingData.channel=punt->channel;
		if(debug) ESP_LOGI(TAG8, "ADD PASARELA PEER");
		pairingStatus=PAIR_PAIRED;
		nvsData.code1 = MAGIC_CODE1;
		memcpy(&(nvsData.data), &pairingData, sizeof(pairingData));
		nvs_saving_task();
		if(debug) ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: EMPAREJAMIENTO");
		xSemaphoreGive(semaforo_envio);
		break;
	}
}

//-----------------------------------------------------------
uint8_t espnow_send(char * mensaje, bool fin, uint8_t _msgType)
{
	if(debug) ESP_LOGI(TAG3, "ESPERANDO SEMAFORO ENVIO");

	if(xSemaphoreTake(semaforo_envio, 3000 / portTICK_PERIOD_MS) == pdFALSE )
	{
		ESP_LOGE(TAG3, "Error esperando a enviar");
		//gotoSleep();
		return ERROR_NOT_PAIRED;
		// a dormir?
	}

	_msgType = _msgType | ((panAddress << PAN_OFFSET) & MASK_PAN);
	char *msg = messType2String(_msgType);
	if(debug) ESP_LOGI(TAG3, "Sending message type = %lld %s",dec2bin(_msgType), msg);
	free(msg);

	mensaje_enviado = true;
	terminar = fin;
	mensajes_sent++;

	int size = strlen(mensaje);
	if (size> 249)
	{
		ESP_LOGE(TAG3, "Error longitud del mensaje demasido grande: %d\n", size);
		return ERROR_MSG_TOO_LARGE;
	}
	struct struct_espnow mensaje_esp;
	mensaje_esp.msgType=_msgType;
	memcpy(mensaje_esp.payload, mensaje, size);
	if(debug) ESP_LOGI(TAG3, "Longitud del mensaje: %d\n", size);
	if(debug) ESP_LOGI(TAG3, "mensaje: %s\n", mensaje);
	esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, size+1);
	espnow_send_cb_t resultado;
	if(debug) ESP_LOGI(TAG3, "ESPERANDO RESULTADO ENVIO");
	if(xQueueReceive(cola_resultado_enviados, &resultado, 200 / portTICK_PERIOD_MS) == pdFALSE)
	{
		ESP_LOGE(TAG3, "Error esperando resultado envío");
		return ERROR_SIN_RESPUESTA;
	}
	// chequar resultado y ver que hacemos
	// también habría que limitar la espera a 100ms por ejemplo
	xSemaphoreGive(semaforo_envio);
	if(debug) ESP_LOGI(TAG3, "LIBERADO SEMAFORO ENVIO: FIN ENVIO");
	if(debug) ESP_LOGI(TAG3, "ESPNOW_STATUS = %d",resultado.status);
	if(resultado.status)return ERROR_ENVIO_ESPNOW; else return ENVIO_OK;
}


//-----------------------------------------------------------

static esp_err_t espnow_init(void) {

	if(debug) ESP_LOGI(TAG, "Pairing request on channel %u\n", espnow_channel);
	// clean esp now
	ESP_ERROR_CHECK(esp_now_deinit());
	// set WiFi channel
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	uint8_t primary = -1;
	wifi_second_chan_t secondary;
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	if(debug) ESP_LOGI(TAG5, "Wifi initialized without problems...\n");
	ESP_ERROR_CHECK(esp_wifi_start());
	if(debug) ESP_LOGI(TAG5, "Wifi started without problems...\n");
	ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
	if(debug) ESP_LOGI(TAG5, "Wifi set mode STA\n");
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
	if(debug) ESP_LOGI(TAG5, "Wifi setting promiscuous = true\n");
	ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &secondary));
	if(debug) ESP_LOGI(TAG5, "Retrieved channel before setting channel: %d\n", primary);
	ESP_ERROR_CHECK(esp_wifi_set_channel(espnow_channel, WIFI_SECOND_CHAN_NONE));
	if(debug) ESP_LOGI(TAG5, "Wifi setting channel  = %d\n", espnow_channel);
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
	if(debug) ESP_LOGI(TAG5, "Wifi setting promiscuous = false\n");
	ESP_ERROR_CHECK(esp_wifi_disconnect());
	if(debug) ESP_LOGI(TAG5, "Wifi  disconnected\n");
	ESP_ERROR_CHECK(esp_now_init());
	if(debug) ESP_LOGI(TAG5, "Wifi esp_now initialized\n");
	ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &secondary));
	if(debug) ESP_LOGI(TAG5, "Retrieved channel after setting it : %d\n", primary);

	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
	ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
	/* Set primary master key. */
	/* Add broadcast peer information to peer list. */
	esp_now_peer_info_t peer;
	peer.channel = espnow_channel;
	peer.ifidx = ESPNOW_WIFI_IF;
	peer.encrypt = false;
	memcpy(peer.peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
	return ESP_OK;
}

static void keep_connection_task(void *pvParameter)
{
	while(1)
	{
		if(debug) ESP_LOGI(TAG2,"Elapsed time = %f",(float)(esp_timer_get_time()-start_time)/1000);
		if((esp_timer_get_time()-start_time)/1000 > timeOut && timeOutEnabled )
		{
			if(debug) ESP_LOGI(TAG2,"SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
			if(debug) ESP_LOGI(TAG2,"millis = %lld limite: %d",esp_timer_get_time(),timeOut);
			gotoSleep();
		}
		switch(pairingStatus) {
		case PAIR_REQUEST:
			if(debug) ESP_LOGI(TAG2,"Pairing request on channel %d" , espnow_channel );

			espnow_init();

			// set pairing data to send to the server
			pairingData.msgType = PAIRING;
			pairingData.id = ESPNOW_DEVICE;

			// send request
			esp_now_send(s_example_broadcast_mac, (uint8_t *) &pairingData, sizeof(pairingData));
			pairingStatus = PAIR_REQUESTED;
			break;

		case PAIR_REQUESTED:
			// time out to allow receiving response from server
			vTaskDelay(100/portTICK_PERIOD_MS);
			if(pairingStatus==PAIR_REQUESTED)
				// time out expired,  try next channel
			{
				espnow_channel ++;
				if (espnow_channel > 11) espnow_channel = 1;
				pairingStatus = PAIR_REQUEST;
			}
			break;

		case PAIR_PAIRED:
			vTaskSuspend(conexion_hand);
			break;
		}
	}
}


void autopairing_init()
{
	start_time = esp_timer_get_time();
	wifi_init();
	xTaskCreate(keep_connection_task, "conexion", 4096, NULL, 1, &conexion_hand);
}

uint32_t read_adc_avg(struct_adclist *ADC_Raw, int chn)
{
	struct_adcread *buf =(struct_adcread *)ADC_Raw->adc_read;
	buf[chn].sum=0;

	buf[chn].adc_buff[buf[chn].AN_i++] = buf[chn].adc_raw;
	if(buf[chn].AN_i == FILTER_LEN)
	{
		buf[chn].AN_i = 0;
	}
	for(int i=0; i<FILTER_LEN; i++)
	{
		buf[chn].sum += buf[chn].adc_buff[i];
	}
	return (buf[chn].sum/FILTER_LEN);
}


uint32_t get_adc_filtered_read(struct_adclist *my_reads, int adc_ch){
	return my_reads->adc_read[adc_ch].adc_filtered;
}
uint32_t get_adc_raw_read(struct_adclist *my_reads, int adc_ch){
	return my_reads->adc_read[adc_ch].adc_raw;
}
uint32_t get_adc_voltage_read(struct_adclist *my_reads, int adc_ch){
	return my_reads->adc_read[adc_ch].voltage;
}


static esp_err_t adc_init(struct_adclist *my_reads){

	//-------------ADC1 TRIG---------------//
	/* Set the GPIO as a push/pull output */
	gpio_reset_pin(TRIGGER_ADC_PIN);
	gpio_set_direction(TRIGGER_ADC_PIN, GPIO_MODE_OUTPUT);
	//-------------ADC1 Init---------------//
	adc_oneshot_unit_init_cfg_t init_config1 = {
			.unit_id = ADC_UNIT_1,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//-------------ADC1 Config---------------//
	adc_oneshot_chan_cfg_t config = {
			.bitwidth = ADC_BITWIDTH_DEFAULT,
			.atten = ADC_ATTEN,
	};
	for(int i=0;i<lengthADC1_CHAN;i++){
		ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channel[i], &config));
	}
	//-------------ADC1 Calibration Init---------------//
	do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN, &adc1_cali_handle);


	unsigned long endwait = convertion_time*lengthADC1_CHAN + (esp_timer_get_time()/1000);

	while ((esp_timer_get_time()/1000) < endwait) {
		gpio_set_level(TRIGGER_ADC_PIN, 1);
		gpio_set_level(BLINK_GPIO, 1);
		for(int i=0;i<my_reads->num;i++){
			ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channel[i], &my_reads->adc_read[i].adc_raw));
			if(debug) ESP_LOGI(TAG2, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, adc_channel[i], my_reads->adc_read[i].adc_raw);
			if (do_calibration) {
				my_reads->adc_read[i].adc_filtered = read_adc_avg(my_reads,i);
				ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, read_adc_avg(my_reads,i), &my_reads->adc_read[i].voltage));
				if(debug) ESP_LOGI(TAG2, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, adc_channel[i], my_reads->adc_read[i].voltage);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	//Tear Down
	gpio_set_level(TRIGGER_ADC_PIN, 0);
	gpio_set_level(BLINK_GPIO, 0);
	ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
	if (do_calibration) {
		adc_calibration_deinit(adc1_cali_handle);
	}
	return ESP_OK;
}


void app_main(void) {

	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// 1.OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// 2.NVS partition contains data in new format and cannot be recognized by this version of code.
		// If this happens, we erase NVS partition and initialize NVS again.
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	set_debug(true);

	nvs_handle_t nvs_handle;
	ret = nvs_open("storage", NVS_READONLY, &nvs_handle);
	if (ret != ESP_OK) {
		if(debug) ESP_LOGI(TAG11, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
	} else {
		if(debug) ESP_LOGI(TAG11, "Open NVS done");
		size_t required_size;
		ret = nvs_get_blob(nvs_handle, "nvs_struct", NULL, &required_size );
		ret = nvs_get_blob(nvs_handle, "nvs_struct", (void *)&nvsData, &required_size);
		switch (ret) {
		case ESP_OK:
			if(debug) ESP_LOGI(TAG11, "Done");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			if(debug) ESP_LOGI(TAG11, "The value is not initialized yet!");
			required_size = sizeof(struct_rtc);
			nvsData.config[1] = timeOut;
			nvsData.config[2] = wakeup_time_sec;
			memset(&(nvsData.data), 0, sizeof(struct_pairing));
			break;
		default :
			if(debug) ESP_LOGI(TAG11, "Error (%s) reading!\n", esp_err_to_name(ret));
		}
	}

	struct_adclist *my_reads = malloc(lengthADC1_CHAN*sizeof(struct_adclist));
	my_reads->num = lengthADC1_CHAN;
	if(debug) ESP_LOGI(TAG,"ADC Length = %d",lengthADC1_CHAN);
	my_reads->adc_read = (struct_adcread *)malloc(my_reads->num*sizeof(struct_adcread));
	for(int i=0;i<my_reads->num;i++){
		my_reads->adc_read[i].AN_i=0;
		my_reads->adc_read[i].adc_raw=0;
		my_reads->adc_read[i].voltage=0;
		my_reads->adc_read[i].sum=0;
		my_reads->adc_read[i].adc_filtered=0;
		for(int j=0;j<FILTER_LEN;j++)my_reads->adc_read[i].adc_buff[j]=0;
	}

	start_time = esp_timer_get_time();
	semaforo_envio = xSemaphoreCreateBinary();
	cola_resultado_enviados = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_send_cb_t));

	if(nvsData.code1 == MAGIC_CODE1){
		// recover information saved in NVS memory
		memcpy(&pairingData, &(nvsData.data), sizeof(pairingData));
		if(debug) ESP_LOGI(TAG, "timeout = %d ",nvsData.config[1]);
		if(debug) ESP_LOGI(TAG, "tsleep = %d ",nvsData.config[2]);
		(nvsData.config[1] == 0) ?  set_timeOut(3000,true) : set_timeOut(nvsData.config[1],true);
		(nvsData.config[2] == 0) ?  set_deepSleep(10) : set_deepSleep(nvsData.config[2]);
		// set WiFi channel
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		if(debug) ESP_LOGI(TAG5, "Wifi initialized without problems...\n");
		ESP_ERROR_CHECK(esp_wifi_start());
		if(debug) ESP_LOGI(TAG5, "Wifi started without problems...\n");
		ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
		if(debug) ESP_LOGI(TAG5, "Wifi set mode STA\n");
		ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
		ESP_ERROR_CHECK(esp_wifi_set_channel(pairingData.channel, WIFI_SECOND_CHAN_NONE));
		if(debug) ESP_LOGI(TAG5, "Wifi setting channel  = %d\n", pairingData.channel);
		ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
		if(debug) ESP_LOGI(TAG5, "Wifi setting promiscuous = false\n");
		ESP_ERROR_CHECK(esp_wifi_disconnect());
		if(debug) ESP_LOGI(TAG5, "Wifi  disconnected\n");
		ESP_ERROR_CHECK(esp_now_init());
		if(debug) ESP_LOGI(TAG5, "Wifi esp_now initialized\n");

		ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
		ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
		ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif

		if(debug) ESP_LOGI(TAG, "Emparejamiento recuperado de la memoria NVS del usuario ");
		if(debug) ESP_LOGI(TAG, "%02X:%02X:%02X:%02X:%02X:%02X", pairingData.macAddr[5],pairingData.macAddr[4],pairingData.macAddr[3],pairingData.macAddr[2],pairingData.macAddr[1],pairingData.macAddr[0]);
		if(debug) ESP_LOGI(TAG, "en el canal %d en %f ms", pairingData.channel, (float)(esp_timer_get_time() - start_time)/1000);

		esp_now_peer_info_t peer;
		peer.channel = pairingData.channel;
		peer.ifidx = ESPNOW_WIFI_IF;
		peer.encrypt = false;
		memcpy(peer.peer_addr, pairingData.macAddr, ESP_NOW_ETH_ALEN);
		ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
		if(debug) ESP_LOGI(TAG8, "ADD PASARELA PEER");
		pairingStatus=PAIR_PAIRED;
		if(debug) ESP_LOGI(TAG8, "LIBERADO SEMAFORO ENVIO: EMPAREJAMIENTO");
		xSemaphoreGive(semaforo_envio);
		vTaskDelay(10);
	}
	else{
		autopairing_init();
	}


	ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

	struct timeval now;
	gettimeofday(&now, NULL);
	int sleep_time_ms = (now.tv_sec - rtcData.sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - rtcData.sleep_enter_time.tv_usec) / 1000;

	if(debug) ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
	while(1){
		if(envio_disponible()){
			cJSON *root, *fmt;
			const esp_partition_t *running = esp_ota_get_running_partition();
			esp_app_desc_t running_app_info;
			char tag[25];
			adc_init(my_reads);
			root = cJSON_CreateObject();
			if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
				cJSON_AddStringToObject(root, "fw_version", running_app_info.version);
			}
			for(int i=0;i<my_reads->num;i++){
				if(debug) ESP_LOGI(TAG, "Serialize readings of channel %d",i);
				sprintf(tag,"Sensor%d", i);
				cJSON_AddItemToObject(root, tag, fmt=cJSON_CreateObject());
				cJSON_AddNumberToObject(fmt, "adc_filtered", get_adc_filtered_read(my_reads,i));
				cJSON_AddNumberToObject(fmt, "adc_voltage", get_adc_voltage_read(my_reads,i));
			}
			char *my_json_string = cJSON_Print(root);
			if(debug) ESP_LOGI(TAG, "my_json_string\n%s",my_json_string);
			espnow_send_check(my_json_string, true, DATA); // hará deepsleep por defecto
			free(my_reads);
			cJSON_Delete(root);
			cJSON_free(my_json_string);
			switch (updateStatus) {
			case THERE_IS_AN_UPDATE_AVAILABLE:
				ESP_ERROR_CHECK(esp_wifi_stop());
				if(debug) ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
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
				gotoSleep();
				break;
			default :
			}
		}
		vTaskDelay(1);
	}
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
	adc_cali_handle_t handle = NULL;
	esp_err_t ret = ESP_FAIL;
	bool calibrated = false;

	if (!calibrated) {
		ESP_LOGI(TAG2, "calibration scheme version is %s", "Curve Fitting");
		adc_cali_curve_fitting_config_t cali_config = {
				.unit_id = unit,
				.atten = atten,
				.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
		if (ret == ESP_OK) {
			calibrated = true;
		}
	}

	*out_handle = handle;
	if (ret == ESP_OK) {
		ESP_LOGI(TAG2, "Calibration Success");
	} else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
		ESP_LOGW(TAG2, "eFuse not burnt, skip software calibration");
	} else {
		ESP_LOGE(TAG2, "Invalid arg or no memory");
	}

	return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
	ESP_LOGI(TAG2, "deregister %s calibration scheme", "Curve Fitting");
	ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

}
