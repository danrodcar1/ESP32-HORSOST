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
#include "esp_adc/adc_continuous.h"

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
#define ADC_ATTEN           ADC_ATTEN_DB_11

static TaskHandle_t s_task_handle;
//ADC1 Channels
static adc_channel_t channel[4] = {ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_4};

float EMA_ALPHA = 0.6;

typedef struct{
	uint32_t adc_raw; 					/*4 bytes*/
	uint32_t adc_filtered;
	int voltage;					/*4 bytes*/
	uint32_t adc_buff[FILTER_LEN];	/*4 bytes*/
	uint32_t sum;						/*4 bytes*/
	int AN_i;						/*4 bytes*/
}struct_adcread;

typedef struct{
	int num;
	struct_adcread *adc_read;
}struct_adclist;
//static_assert(sizeof(struct_adclist) == 8);

unsigned long millis = 0;

static uint8_t s_led_state = 0;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = ADC_UNIT_1;
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
	if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
		return false;
	}
	return true;
}

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
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(TRIGGER_ADC_PIN, 1);
        gpio_set_level(BLINK_GPIO, 1);
        conv_on = true;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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
				//waiting=false;
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

inline void adc_multisampling(struct_adclist *my_reads,adc_digi_output_data_t *p){
	switch ( p->type2.channel )
	{
	case 0:
		my_reads->adc_read[0].adc_raw=p->type2.data;
		my_reads->adc_read[0].adc_filtered = EMA_ALPHA * my_reads->adc_read[0].adc_raw + (1 - EMA_ALPHA) * my_reads->adc_read[0].adc_filtered;
		break;
	case 1:
		my_reads->adc_read[1].adc_raw=p->type2.data;
		my_reads->adc_read[1].adc_filtered = EMA_ALPHA * my_reads->adc_read[0].adc_raw + (1 - EMA_ALPHA) * my_reads->adc_read[1].adc_filtered;
		break;
	case 2:
		my_reads->adc_read[2].adc_raw=p->type2.data;
		my_reads->adc_read[2].adc_filtered = EMA_ALPHA * my_reads->adc_read[0].adc_raw + (1 - EMA_ALPHA) * my_reads->adc_read[2].adc_filtered;
		break;
	case 4:
		my_reads->adc_read[3].adc_raw=p->type2.data;
		my_reads->adc_read[3].adc_filtered = EMA_ALPHA * my_reads->adc_read[0].adc_raw + (1 - EMA_ALPHA) * my_reads->adc_read[3].adc_filtered;
		//my_reads->adc_read[3].adc_filtered=read_adc_avg(my_reads,3);
		break;
	}
}

void init_adc_dma_mode(){
	esp_err_t ret;
	uint32_t ret_num = 0;
	uint8_t result[READ_LEN] = {0};
	memset(result, 0xcc, READ_LEN);


	s_task_handle = xTaskGetCurrentTaskHandle();

	adc_continuous_handle_t handle = NULL;

	uint8_t lengthADC1_CHAN = sizeof(channel) / sizeof(adc_channel_t);
	continuous_adc_init(channel, lengthADC1_CHAN, &handle);

	struct_adclist *my_reads = malloc(lengthADC1_CHAN*sizeof(struct_adclist));
	my_reads->num = lengthADC1_CHAN;
	ESP_LOGI(TAG,"ADC Length = %d",lengthADC1_CHAN);
	my_reads->adc_read = (struct_adcread *)malloc(my_reads->num*sizeof(struct_adcread));
	for(int i=0;i<my_reads->num;i++){
		my_reads->adc_read[i].AN_i=0;
		my_reads->adc_read[i].adc_raw=0;
		my_reads->adc_read[i].voltage=0;
		my_reads->adc_read[i].sum=0;
		my_reads->adc_read[i].adc_filtered=0;
		for(int j=0;j<FILTER_LEN;j++)my_reads->adc_read[i].adc_buff[j]=0;
	}

	adc_continuous_evt_cbs_t cbs = {
			.on_conv_done = s_conv_done_cb,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
	ESP_ERROR_CHECK(adc_continuous_start(handle));

	while(1) {

		/**
		 * This is to show you the way to use the ADC continuous mode driver event callback.
		 * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
		 * However in this example, the data processing (print) is slow, so you barely block here.
		 *
		 * Without using this event callback (to notify this task), you can still just call
		 * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
		 */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		while (conv_on) {
			ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
			if (ret == ESP_OK) {
				ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32, ret, ret_num);
				for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
					adc_digi_output_data_t *p = (void*)&result[i];
					if (check_valid_data(p)) {
						//Multisampling each channel and take average reading
						adc_multisampling(my_reads,p);

						ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Raw_value: %x", 1, p->type2.channel, p->type2.data);
						ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Filtered_value: %lu", 1, p->type2.channel, my_reads->adc_read[0].adc_filtered);

					} else {
						ESP_LOGI(TAG, "Invalid data");
					}
				}
				/**
				 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
				 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
				 * usually you don't need this delay (as this task will block for a while).
				 */
				vTaskDelay(1);
			} else if (ret == ESP_ERR_TIMEOUT) {
				//We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
				break;
			}
		}
	}

	ESP_ERROR_CHECK(adc_continuous_stop(handle));
	ESP_ERROR_CHECK(adc_continuous_deinit(handle));
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
	xTaskCreate(&blinky, "blinky", 1024,NULL,2,NULL );
	//adc_init();
	init_adc_dma_mode();
	/* Configure the peripheral according to the LED type */

	//adc_init(NULL);
	   // xTaskCreate(TaskBlink, "task1", 128, NULL, 1, NULL );
}
