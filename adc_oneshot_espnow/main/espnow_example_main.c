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
#include <string.h>
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

// DEEP SLEEP VARS
int wakeup_time_sec;
int mensajes_sent = 0;
int panAddress = 1;
bool esperando = false;
bool terminar = false;
bool mensaje_enviado = false;
bool debug = false;
unsigned long timeOut;
unsigned long start_time;
bool timeOutEnabled;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

#define CANAL 6

static int espnow_channel = CANAL;

#define ESPNOW_MAXDELAY 512

#define ERROR_NOT_PAIRED    1
#define ERROR_MSG_TOO_LARGE 2
#define ERROR_SIN_RESPUESTA 3
#define ERROR_ENVIO_ESPNOW  4
#define ENVIO_OK            0

TaskHandle_t adcTaskHandle = NULL;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;

static const char *TAG  = "* mainApp";
static const char *TAG2 = "* adc_reads";
static const char *TAG3 = "* task conexion";
static const char *TAG4 = "* funcion envio";
static const char *TAG8 = "* funcion recivo";
static const char *TAG5 = "* init espnow";
static const char *TAG6 = "* callbacks espnow";
static const char *TAG7 = "* deep sleep";

static QueueHandle_t cola_resultado_enviados;
static SemaphoreHandle_t semaforo_envio;

typedef struct {
	uint8_t mac_addr[ESP_NOW_ETH_ALEN];
	esp_now_send_status_t status;
} espnow_send_cb_t;

PairingStatus pairingStatus=PAIR_REQUEST;
struct struct_pairing pairingData;

TaskHandle_t conexion_hand = NULL;

struct struct_pairing pairingData;

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
// VARIABLES

bool conv_on;

#define BLINK_GPIO 10
#define TRIGGER_ADC_PIN 6
/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
#define READ_LEN   256
#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define FILTER_LEN  15
#define ADC_ATTEN           ADC_ATTEN_DB_6

static TaskHandle_t s_task_handle;
//ADC1 Channels
const int adc_channel[1] = {ADC_CHANNEL_0};
//static adc_channel_t adc_channel[4] = {ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_4};
uint8_t lengthADC1_CHAN = sizeof(adc_channel) / sizeof(adc_channel_t);

float EMA_ALPHA = 0.6;

typedef struct{
	int adc_raw; 					/*4 bytes*/
	int voltage;					/*4 bytes*/
	uint32_t adc_buff[FILTER_LEN];	/*4 bytes*/
	int sum;						/*4 bytes*/
	uint32_t adc_filtered;
	int AN_i;						/*4 bytes*/
}struct_adcread;

typedef struct{
	char *topic;
	char *payload;
	uint8_t macAddr[6];
}struct_espnow_rcv_msg;

typedef struct{
	int num;
	struct_adcread *adc_read;
}struct_adclist;
//static_assert(sizeof(struct_adclist) == 8);


unsigned long convertion_time = 200;
unsigned long previous_conv_time = 0;
unsigned long current_conv_time = 0;


#define OTA_URL_SIZE 256
#define HASH_LEN 32
#define FIRMWARE_UPGRADE_URL "https://huertociencias.uma.es/esp8266-ota-update"

static uint8_t s_led_state = 0;

// Function prototypes
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
uint8_t espnow_send(char * mensaje, bool fin, uint8_t _msgType);

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

//void simple_ota_example_task(void *pvParameter)
//{
//    ESP_LOGI(TAG, "Starting OTA example task");
//#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
//    esp_netif_t *netif = get_example_netif_from_desc(bind_interface_name);
//    if (netif == NULL) {
//        ESP_LOGE(TAG, "Can't find netif from interface description");
//        abort();
//    }
//    struct ifreq ifr;
//    esp_netif_get_netif_impl_name(netif, ifr.ifr_name);
//    ESP_LOGI(TAG, "Bind interface name is %s", ifr.ifr_name);
//#endif
//    esp_http_client_config_t config = {
//        .url = CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL,
//#ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
//        .crt_bundle_attach = esp_crt_bundle_attach,
//#else
//        .cert_pem = (char *)server_cert_pem_start,
//#endif /* CONFIG_EXAMPLE_USE_CERT_BUNDLE */
//        .event_handler = _http_event_handler,
//        .keep_alive_enable = true,
//#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_BIND_IF
//        .if_name = &ifr,
//#endif
//    };
//
//#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
//    char url_buf[OTA_URL_SIZE];
//    if (strcmp(config.url, "FROM_STDIN") == 0) {
//        example_configure_stdin_stdout();
//        fgets(url_buf, OTA_URL_SIZE, stdin);
//        int len = strlen(url_buf);
//        url_buf[len - 1] = '\0';
//        config.url = url_buf;
//    } else {
//        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
//        abort();
//    }
//#endif
//
//#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
//    config.skip_cert_common_name_check = true;
//#endif
//
//    esp_https_ota_config_t ota_config = {
//        .http_config = &config,
//    };
//    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
//    esp_err_t ret = esp_https_ota(&ota_config);
//    if (ret == ESP_OK) {
//        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
//        esp_restart();
//    } else {
//        ESP_LOGE(TAG, "Firmware upgrade failed");
//    }
//    while (1) {
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//}

void blinky(void *pvParameter)
{
	gpio_reset_pin(TRIGGER_ADC_PIN);
	gpio_reset_pin(BLINK_GPIO);

	/* Set the GPIO as a push/pull output */
	gpio_set_direction(TRIGGER_ADC_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	while(1) {
		/* Blink off (output low) */
		gpio_set_level(TRIGGER_ADC_PIN, 0);
		gpio_set_level(BLINK_GPIO, 0);
		conv_on = false;
		vTaskDelay(convertion_time / portTICK_PERIOD_MS);
		/* Blink on (output high) */
		gpio_set_level(TRIGGER_ADC_PIN, 1);
		gpio_set_level(BLINK_GPIO, 1);
		conv_on = true;
		vTaskDelay(convertion_time / portTICK_PERIOD_MS);
	}
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
void set_deepSleep(int _wakeup_time_sec)
{
	wakeup_time_sec=_wakeup_time_sec;
}

void set_timeOut(unsigned long _timeOut, bool _enable)
{
 timeOut = _timeOut;
 timeOutEnabled = _enable;
}

//--------------------------------------------------------
void gotoSleep() {
	// add some randomness to avoid collisions with multiple devices
	if(debug) ESP_LOGI(TAG7, "Apaga y vamonos");
	ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
	// enter deep sleep
	esp_deep_sleep_start();
}

static void get_mac_address()
{
    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    ESP_LOGI("MAC address", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void set_mac_address(uint8_t *mac)
{
    esp_err_t err = esp_wifi_set_mac(ESP_IF_WIFI_STA, mac);
    if (err == ESP_OK) {
        ESP_LOGI("MAC address", "MAC address successfully set to %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        ESP_LOGE("MAC address", "Failed to set MAC address");
    }
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

	if(debug) ESP_LOGI(TAG4, "ENVIO ESPNOW status: %d", status);
	if(debug) ESP_LOGI(TAG4, "ENVIO ESPNOW status: %s", (status)?"ERROR":"OK");
	if(debug) ESP_LOGI(TAG4, "MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[5],mac_addr[4],mac_addr[3],mac_addr[2],mac_addr[1],mac_addr[0]);

	if(pairingStatus==PAIR_PAIRED && mensaje_enviado)  // será un mensaje a la pasarela, se podría comprobar la mac
	{
		mensaje_enviado = false;

		//if (terminar && !esperando) gotoSleep();
	}

	if(xQueueSend(cola_resultado_enviados,&resultado, ESPNOW_MAXDELAY) != pdTRUE)ESP_LOGW(TAG, "Send send queue fail");
}

static esp_err_t mqtt_process_msg(struct_espnow_rcv_msg *my_msg){
	if(debug) ESP_LOGI(TAG8, "topic: %s", my_msg->topic);
	cJSON *root2 = cJSON_Parse(my_msg->payload);
	if(strcmp (my_msg->topic,"config") == 0){
		if(debug) ESP_LOGI(TAG8, "payload: %s", my_msg->payload);
		if(debug) ESP_LOGI(TAG8, "Deserialize payload.....");
		int tsleep = cJSON_GetObjectItem(root2,"sleep")->valueint;
		int tout = cJSON_GetObjectItem(root2,"timeout")->valueint;
		if(debug) ESP_LOGI(TAG8, "tsleep = %d",tsleep);
		if(debug) ESP_LOGI(TAG8, "tout = %d",tout);
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
	struct struct_pairing *punt = (struct struct_pairing*) data;
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

static void mantener_conexion(void *pvParameter)
{
	while(1)
	{
		if(debug) ESP_LOGI(TAG2,"Elapsed time = %f",(float)(esp_timer_get_time()-start_time)/1000);
		if((esp_timer_get_time()-start_time)/1000 > timeOut && timeOutEnabled )
		{
			if(debug) ESP_LOGI(TAG2,"SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
			if(debug) ESP_LOGI(TAG2,"millis = %lld limite: %ld",esp_timer_get_time(),timeOut);
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
				if (espnow_channel > 11) espnow_channel = 0;
				pairingStatus = PAIR_REQUEST;
			}
			break;

		case PAIR_PAIRED:
			vTaskSuspend(NULL);
			break;
		}
	}
}


void autopairing_init()
{
	start_time = esp_timer_get_time();
	wifi_init();
	semaforo_envio = xSemaphoreCreateBinary();
	cola_resultado_enviados = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_send_cb_t));
	xTaskCreate(mantener_conexion, "conexion", 4096, NULL, 1, &conexion_hand);
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

	while ((esp_timer_get_time()/1000) < endwait) {//
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
	uint8_t resultado;
	cJSON *root, *fmt;
	char tag[25];

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

	set_debug(true);
	set_deepSleep(10);
	set_timeOut(3000,true); // tiempo máximo
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	struct timeval now;
	gettimeofday(&now, NULL);
	int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

	autopairing_init();

	while(1){
		if(envio_disponible()){
			adc_init(my_reads);
			root = cJSON_CreateObject();

			for(int i=0;i<my_reads->num;i++){
				if(debug) ESP_LOGI(TAG, "Serialize readings of channel %d",i);
				sprintf(tag,"Sensor%d", i);
				cJSON_AddItemToObject(root, tag, fmt=cJSON_CreateObject());
				cJSON_AddNumberToObject(fmt, "adc_voltage", get_adc_voltage_read(my_reads,i));
			}

			char *my_json_string = cJSON_Print(root);
			if(debug) ESP_LOGI(TAG, "my_json_string\n%s",my_json_string);
			espnow_send_check(my_json_string, true, DATA); // hará deepsleep por defecto
			free(my_reads);
			cJSON_Delete(root);
			cJSON_free(my_json_string);
			break;
		}
		vTaskDelay(1);
	}
	ESP_LOGI(TAG, "FIN DEL MAIN APP");
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
