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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "espnow_example.h"

#define ESPNOW_MAXDELAY 512

TaskHandle_t adcTaskHandle = NULL;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;

static const char *TAG = "espnow_example";
static const char *TAG2 = "adc_reads";

static struct_pairing pairingData;
static PairingStatus pairingStatus;

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// VARIABLES

bool waiting;

#define BLINK_GPIO 10

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels

const int ADC1_CHAN[]={ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_4};
int lengthADC1_CHAN= sizeof(ADC1_CHAN) / sizeof(ADC1_CHAN[0]);

#define FILTER_LEN  15

typedef struct{
	int adc_raw; 					/*4 bytes*/
	int voltage;					/*4 bytes*/
	uint32_t adc_buff[FILTER_LEN];	/*4 bytes*/
	int sum;						/*4 bytes*/
	int AN_i;						/*4 bytes*/
}struct_adcread;

typedef struct{
	int num;
	struct_adcread *adc_read;
}struct_adclist;
//static_assert(sizeof(struct_adclist) == 8);

uint32_t AN_Buffer[4][FILTER_LEN] = {0};
int AN_i = 0;
int AN_Raw = 0;
int AN_Filtered = 0;

#define ADC_ATTEN           ADC_ATTEN_DB_11

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

static uint8_t s_led_state = 0;

void blinky(void *pvParameter)
{
	gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr,	esp_now_send_status_t status) {
	example_espnow_event_t evt;
	example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

	if (mac_addr == NULL) {
		ESP_LOGE(TAG, "Send cb arg error");
		return;
	}

	evt.id = EXAMPLE_ESPNOW_SEND_CB;
	memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
	send_cb->status = status;
	if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
		ESP_LOGW(TAG, "Send send queue fail");
	}
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
	example_espnow_event_t evt;
	example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
	uint8_t *mac_addr = recv_info->src_addr;

	if (mac_addr == NULL || data == NULL || len <= 0) {
		ESP_LOGE(TAG, "Receive cb arg error");
		return;
	}

	evt.id = EXAMPLE_ESPNOW_RECV_CB;
	memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
	recv_cb->data = malloc(len);
	if (recv_cb->data == NULL) {
		ESP_LOGE(TAG, "Malloc receive data fail");
		return;
	}
	memcpy(recv_cb->data, data, len);
	recv_cb->data_len = len;
	if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
		ESP_LOGW(TAG, "Send receive queue fail");
		free(recv_cb->data);
	}

	ESP_LOGI(TAG,"Size of message : %d from "MACSTR"\n",recv_cb->data_len,MAC2STR(recv_cb->mac_addr));
}

static void example_espnow_task(void *pvParameter) {
	example_espnow_event_t evt;
	char *topic;
	char *payload;

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "Start sending broadcast data");

	/* Start sending broadcast ESPNOW data. */
	pairingData.id=BOARD_ID;
	pairingData.channel=CONFIG_ESPNOW_CHANNEL;
	if (esp_now_send(s_example_broadcast_mac, (uint8_t*) &pairingData, sizeof(pairingData)) != ESP_OK) {
		ESP_LOGE(TAG, "Send error");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
		vTaskDelete(NULL);
	}

	int64_t Timer7 = esp_timer_get_time();
	printf("Timer: %lld μs\n", Timer7);
	while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
		switch (evt.id) {
		case EXAMPLE_ESPNOW_SEND_CB: {
			example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

			ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			struct_espnow mensaje_esp;
			mensaje_esp.msgType = (uint8_t)DATA;
			char msg[]="{\"messageSend\":true}";
			memcpy(mensaje_esp.payload, msg, strlen(msg));
			//Send the next data after the previous data is sent.
			if (esp_now_send(s_example_broadcast_mac, (uint8_t *) &mensaje_esp,	strlen(msg)+1) != ESP_OK) {
				ESP_LOGE(TAG, "Send error");
				vSemaphoreDelete(s_example_espnow_queue);
				esp_now_deinit();
				vTaskDelete(NULL);
			}

			break;
		}
		case EXAMPLE_ESPNOW_RECV_CB: {
			example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
			struct_pairing *buf = (struct_pairing *)recv_cb->data;
			uint8_t type = buf->msgType;
			pairingData.id = buf->id;
			ESP_LOGI(TAG,"message type = "BYTE_TO_BINARY_PATTERN"\n",BYTE_TO_BINARY(type));
			free(recv_cb->data);
			switch(type & 0b00000011){
			case NODATA:
				waiting=false;
				ESP_LOGI(TAG,"No hay mensajes MQTT");
				break;
			case DATA:
				ESP_LOGI(TAG,"Mensaje recibido MQTT");
				for(uint8_t i=0; i<recv_cb->data_len; i++) if(recv_cb->data[i]=='|') break;
				break;
			case PAIRING:
				//memcpy(&pairingData, (struct_pairing*)recv_cb->data, sizeof(pairingData));
				ESP_LOGI(TAG,"Pairing ID: %d",pairingData.id);
				/* If MAC address does not exist in peer list, add it to peer list. */
				if(pairingData.id==0){
					if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
						esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
						if (peer == NULL) {
							ESP_LOGE(TAG, "Malloc peer information fail");
							vSemaphoreDelete(s_example_espnow_queue);
							esp_now_deinit();
							vTaskDelete(NULL);
						}
						memset(peer, 0, sizeof(esp_now_peer_info_t));
						peer->channel = CONFIG_ESPNOW_CHANNEL;
						peer->ifidx = ESPNOW_WIFI_IF;
						peer->encrypt = false;
						//				memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
						memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
						ESP_ERROR_CHECK( esp_now_add_peer(peer) );
						free(peer);
						pairingStatus = PAIR_PAIRED ;            // set the pairing status
						ESP_LOGI(TAG,"Peer added");
					}
				}
				break;
			}
		}
		}
	}
}

static esp_err_t example_espnow_init(void) {
	/* Create a queue capable of containing ESPNOW_QUEUE_SIZE example_espnow_event_t struct. */
	s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
	if (s_example_espnow_queue == NULL) {
		ESP_LOGE(TAG, "Create mutex fail");
		return ESP_FAIL;
	}

	/* Initialize ESPNOW and register sending and receiving callback function. */
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
	ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif
	/* Set primary master key. */
	//ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

	/* Add broadcast peer information to peer list. */
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (peer == NULL) {
		ESP_LOGE(TAG, "Malloc peer information fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
		return ESP_FAIL;
	}
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK(esp_now_add_peer(peer));
	free(peer);

	xTaskCreate(example_espnow_task, "example_espnow_task", 2048, NULL, 4, NULL);

	return ESP_OK;
}


int readADC_Avg(struct_adclist *ADC_Raw, int chn)
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

static void adc_task(void *pvParameter){
	struct_adclist *my_reads = (struct_adclist *)pvParameter;
	while (1) {
		for(int i=0;i<my_reads->num;i++){
			ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN[i], &my_reads->adc_read[i].adc_raw));
			ESP_LOGI(TAG2, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN[i], my_reads->adc_read[i].adc_raw);
			if (do_calibration) {
				ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, readADC_Avg(my_reads,i), &my_reads->adc_read[i].voltage));
				ESP_LOGI(TAG2, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC1_CHAN[i], my_reads->adc_read[i].voltage);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	//Tear Down
	ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
	if (do_calibration) {
		free(my_reads);
		example_adc_calibration_deinit(adc1_cali_handle);
	}
}

static esp_err_t adc_init(void){

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
		ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN[i], &config));
	}
	//-------------ADC1 Calibration Init---------------//
	do_calibration = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN, &adc1_cali_handle);
	struct_adclist *my_reads = malloc(lengthADC1_CHAN*sizeof(struct_adclist));
	my_reads->num = lengthADC1_CHAN;
	my_reads->adc_read = (struct_adcread *)malloc(my_reads->num*sizeof(struct_adcread));
	for(int i=0;i<my_reads->num;i++){
		my_reads->adc_read[i].AN_i=0;
		my_reads->adc_read[i].adc_raw=0;
		my_reads->adc_read[i].voltage=0;
		my_reads->adc_read[i].sum=0;
		for(int j=0;j<FILTER_LEN;j++)my_reads->adc_read[i].adc_buff[j]=0;
	}
	xTaskCreate(&adc_task, "adc_init", 4096, my_reads, 5, &adcTaskHandle);
	return ESP_OK;
}

void app_main(void) {
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	//example_wifi_init();
	//example_espnow_init();
	adc_init();

	/* Configure the peripheral according to the LED type */
	//xTaskCreate(&blinky, "blinky", 512,NULL,1,NULL );
	//adc_init(NULL);
	   // xTaskCreate(TaskBlink, "task1", 128, NULL, 1, NULL );
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
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

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(TAG2, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

}
