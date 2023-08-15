#include <stdio.h>
#include "esp_log.h"
//------- ESP32 HEADERS .- ONESHOT ADC ----------//
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
//------- ESP32 HEADERS .- FREERTOS ----------//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//------- ESP32 HEADERS .- GPIO ----------//
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ADConeshot_component.h"

static const char *TAG = "* adc_reads";
/*---------------------------------------------------------------
        ADC variables
        - Create task handle to know when we have to point the ADC conversor
        - Do the same with the calibration unit
        - Set up adc channels
        - Set up convertion-time studied for one analog sensor previously
---------------------------------------------------------------*/



float EMA_ALPHA = 0.6;

unsigned long convertion_time = 200;
//ADC1 Channels
TaskHandle_t adcTaskHandle = NULL;
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration;
int *_adc_channel;
//static adc_channel_t _adc_channel[4] = {ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_2,ADC_CHANNEL_4};
uint8_t _lengthADC1_CHAN;

esp_err_t set_adc_channel(int* ptr, int size){
	_adc_channel = malloc(size*sizeof(*_adc_channel));
	if(ptr == NULL)ESP_LOGW(TAG,"Memory not allocated");
	else{
		ESP_LOGI(TAG,"Memory successfully allocated using malloc");
		for(int i = 0; i < size; i++) _adc_channel[i] = ptr[i];
//		free(_adc_channel);
	}
	return ESP_OK;
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
		ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
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
		ESP_LOGI(TAG, "Calibration Success");
	} else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
		ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
	} else {
		ESP_LOGE(TAG, "Invalid arg or no memory");
	}
	return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
	ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
	ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
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


esp_err_t adc_init(struct_adclist *my_reads){

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
	for(int i=0;i<_lengthADC1_CHAN;i++){
		ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, _adc_channel[i], &config));
	}
	//-------------ADC1 Calibration Init---------------//
	do_calibration = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN, &adc1_cali_handle);


	unsigned long endwait = convertion_time*_lengthADC1_CHAN + (esp_timer_get_time()/1000);

	while ((esp_timer_get_time()/1000) < endwait) {
		gpio_set_level(TRIGGER_ADC_PIN, 1);
		gpio_set_level(BLINK_GPIO, 1);
		for(int i=0;i<my_reads->num;i++){
			ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, _adc_channel[i], &my_reads->adc_read[i].adc_raw));
			ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, _adc_channel[i], my_reads->adc_read[i].adc_raw);
			if (do_calibration) {
				my_reads->adc_read[i].adc_filtered = read_adc_avg(my_reads,i);
				ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, read_adc_avg(my_reads,i), &my_reads->adc_read[i].voltage));
				ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, _adc_channel[i], my_reads->adc_read[i].voltage);
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


