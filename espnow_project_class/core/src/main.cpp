#include "AUTOpairing.h"
#include "OTAupdate.h"
#include "cJSON.h"
#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE



#define LOG_TAG "main"
static AUTOpairing_t clienteAP;
static OTAupdate_t OTAupdate;

typedef struct{
	uint8_t tsleep;
	uint8_t pan;
	uint16_t timeout;
}struct_config;

struct_config strConfig;

UpdateStatus updateStatus = NO_UPDATE_FOUND;


void mqtt_process_msg(struct_espnow_rcv_msg *my_msg){
	ESP_LOGI("* mqtt process", "topic: %s", my_msg->topic);
	cJSON *root2 = cJSON_Parse(my_msg->payload);
	if(strcmp (my_msg->topic,"config") == 0){
		ESP_LOGI("* mqtt process", "payload: %s", my_msg->payload);
		ESP_LOGI("* mqtt process", "Deserialize payload.....");
		cJSON *sleep = cJSON_GetObjectItem(root2,"sleep");
		cJSON *timeout = cJSON_GetObjectItem(root2,"timeout");
		cJSON *pan = cJSON_GetObjectItem(root2,"pan");
		uint16_t config[MAX_CONFIG_SIZE]; // max config size
		if(timeout) config[1] = timeout->valueint;
		if(sleep) config[2] = sleep->valueint;
		if(pan) config[3] = pan->valueint;
		ESP_LOGI("* mqtt process", "tsleep = %d",config[1]);
		ESP_LOGI("* mqtt process", "timeout = %d",config[2]);
		ESP_LOGI("* mqtt process", "PAN_ID = %d",config[3]);
		clienteAP.set_config(config);
	}
	if(strcmp (my_msg->topic,"update") == 0){
		updateStatus = THERE_IS_AN_UPDATE_AVAILABLE;

	}
	free(my_msg->topic);
	free(my_msg->payload);
	cJSON_Delete(root2);
//	return ESP_OK;
}


extern "C" void app_main(void){
	clienteAP.init_config_size(sizeof(strConfig));
	if(!clienteAP.get_config((uint16_t*)&strConfig)){
		strConfig.timeout = 3000;
		strConfig.tsleep = 10;
	}
	clienteAP.set_timeOut(strConfig.timeout,true); // tiempo máximo
	clienteAP.set_deepSleep(strConfig.tsleep);  //tiempo dormido en segundos
	clienteAP.set_channel(6);  // canal donde empieza el scaneo
	clienteAP.set_callback(mqtt_process_msg);  //por defecto a NULL -> no se llama a ninguna función
	clienteAP.begin();
	while(true){
		if(!clienteAP.emparejado())clienteAP.start_connection_task();
		if(clienteAP.envio_disponible() == true){
			ESP_LOGI(LOG_TAG,"envio disponible");
			char mensaje[256];
			sprintf(mensaje, "{\"topic\":\"datos\",\"temp\":%4.2f, \"hum\":%4.2f }", 24.2, 46.0);
			clienteAP.espnow_send_check(mensaje); // hará deepsleep por defecto
			switch (updateStatus) {
			case THERE_IS_AN_UPDATE_AVAILABLE:
				ESP_ERROR_CHECK(esp_wifi_stop());
				ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
				OTAupdate.wifi_init_sta();
				/*
				 * Ensure to disable any WiFi power save mode, this allows best throughput
				 * and hence timings for overall OTA operation.
				 */
				esp_wifi_set_ps(WIFI_PS_NONE);
				OTAupdate.advance_ota_task();
				break;
			case NO_UPDATE_FOUND:
				// get deep sleep enter time
				//						gotoSleep();
				break;
			}
		}

		vTaskDelay(1);
	}
}
