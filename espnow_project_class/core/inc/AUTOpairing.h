#pragma once

//------- C HEADERS ----------//
#include "stdint.h"
#include <string>
//#include "string.h"
//------- ESP32 HEADERS .- FREERTOS ----------//
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//------- ESP32 HEADERS .- WIFI & ESPNOW ----------//
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
//------- ESP32 HEADERS .- FLASH ----------//
#include "nvs_flash.h"
//#include "espnow_example.h"
#include "AUTOpairing_common.h"

using namespace std;

class mensajeMQTT_t
{
public:
//	string topic;
//	string payload;
	struct_espnow_rcv_msg my_msg;
	mensajeMQTT_t ( struct_espnow_rcv_msg *msg_recv)
	{
		my_msg.topic=msg_recv->topic;
		my_msg.payload=msg_recv->payload;
	}
};


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
TaskHandle_t conexion_hand = NULL;



static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

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

#define pdSECOND pdMS_TO_TICKS(1000)

#ifndef MAX_CONFIG_SIZE
#define MAX_CONFIG_SIZE 64
#endif

class AUTOpairing_t
{
	static AUTOpairing_t *this_object;
	typedef enum {
		PAIR_REQUEST,
		PAIR_REQUESTED,
		PAIR_PAIRED,
	}PairingStatus;
	typedef struct{
		uint16_t code1;
		uint16_t code2;
		struct_pairing data;
		uint16_t config[MAX_CONFIG_SIZE]; // max config size
	}struct_nvs;
	typedef struct{
		struct timeval sleep_enter_time;
	}struct_rtc;

	static RTC_DATA_ATTR struct_rtc rtcData;
	// Others control vars
	int config_size; //check
	int mensajes_sent;
	PairingStatus pairingStatus;
	struct_nvs nvsData;
	struct_pairing pairingData;
	bool mensaje_enviado; //check
	bool terminar;
	unsigned long start_time;
	unsigned long previousMillis_scanChannel;    // will store last time channel was scanned
	bool esperando;
	bool rtc_init;

	void (*user_callback)(struct_espnow_rcv_msg*);

	// User control vars
	bool debug;
	unsigned long timeOut;
	bool timeOutEnabled;
	bool nvs_start;
	uint8_t espnow_channel;
	int wakeup_time_sec;
	int panAddress;

public:
	AUTOpairing_t()
	{
		this_object = this;
		pairingStatus = PAIR_REQUEST;
		mensaje_enviado=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
		terminar=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
		esperando=false;
		timeOutEnabled=true;
		nvs_start=false;
		rtc_init=false;
		wakeup_time_sec = 10;   // tiempo dormido en segundos
		panAddress = 1;
		config_size = MAX_CONFIG_SIZE;
		start_time=0;  // para controlar el tiempo de escaneo
		previousMillis_scanChannel=0;
		timeOut=3000;
		debug=true;
		espnow_channel = 6;  // canal para empezar a escanear
		mensajes_sent=0;
	}

	//-----------------------------------------------------------
	void set_pan(uint8_t _pan = 1){ panAddress = _pan;	}
	//-----------------------------------------------------------
	int get_pan(){ return panAddress; }
	//-----------------------------------------------------------
	void set_timeOut(unsigned long _timeOut = 3000, bool _enable = true){  timeOut = _timeOut; timeOutEnabled = _enable; }
	//-----------------------------------------------------------
	void set_channel(uint8_t _channel = 6){ espnow_channel = _channel; }
	//-----------------------------------------------------------
	void set_FLASH(bool _nvs_start = false){ nvs_start = _nvs_start; }
	//-----------------------------------------------------------
	void set_debug(bool _debug = true){ debug = _debug; }
	//-----------------------------------------------------------
	void set_deepSleep(int _wakeup_time_sec = 10){ wakeup_time_sec = _wakeup_time_sec;	}
	//-----------------------------------------------------------
	bool init_config_size(uint8_t size)
	{
		config_size = size;
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
				required_size = sizeof(struct_nvs);
				nvsData.config[1] = timeOut;
				nvsData.config[2] = wakeup_time_sec;
				memset(&(nvsData.data), 0, sizeof(struct_pairing));
				break;
			default :
				if(debug) ESP_LOGI(TAG11, "Error (%s) reading!\n", esp_err_to_name(ret));
			}
		}
		if (size>MAX_CONFIG_SIZE)
		{
			ESP_LOGI(TAG11,"Espacio reservado en FLASH demasiado pequeño: %d\n Por favor incremente el valor MAX_CONFIG_SIZE",MAX_CONFIG_SIZE );
			return false;
		}
		else
			return true;
	}
	//-----------------------------------------------------------
	 bool get_config(uint16_t* config)
	{
	  //init check FLASH

	  if(nvsData.code2==MAGIC_CODE2)
	  {
	    memcpy(config, &(nvsData.config), config_size);
	    if(debug) ESP_LOGI(TAG11,"Configuración leída de FLASH");
	    return true;
	  }
	  else
	  {
	    if(debug) ESP_LOGI("TAG11","Sin configuración en FLASH");
	    return false;
	  }
	}
	//-----------------------------------------------------------
	void set_config(uint16_t* config){
		nvsData.code2 = MAGIC_CODE2;
		memcpy(&(nvsData.config), config, config_size);
		nvs_saving_task();
	}
	//-----------------------------------------------------------
	void nvs_saving_task(){
		// Save in NVS
		esp_err_t ret;
		nvs_handle_t nvs_handle;
		ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
		if (ret != ESP_OK) {
			if(debug) ESP_LOGI(TAG11, "Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
		} else {
			if(debug) ESP_LOGI(TAG11, "Open NVS done\n");
			if(debug) ESP_LOGI(TAG11, "Adding text to NVS Struct... ");
			ret = nvs_set_blob(nvs_handle, "nvs_struct", (const void*)&nvsData, sizeof(struct_nvs));
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
	//---------------------------------------------------
	/* WiFi should start before using ESPNOW */
	void wifi_espnow_init(void) {
		ESP_ERROR_CHECK(esp_netif_init());
		ESP_ERROR_CHECK(esp_event_loop_create_default());
		wifi_init_config_t cfg = esp_wifi_init_config_default();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
		ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
		ESP_ERROR_CHECK(esp_wifi_start());

	#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
		ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
	#endif
	}
	//----------------------------------------------------
	void begin(void){


		if(debug) ESP_LOGI(TAG, "Comienza AUTOpairing...");
		previousMillis_scanChannel=0;
		start_time=esp_timer_get_time()/1000;
		semaforo_envio = xSemaphoreCreateBinary();
		cola_resultado_enviados = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_send_cb_t));

		if(nvsData.code1 == MAGIC_CODE1){
			// recover information saved in NVS memory
			memcpy(&pairingData, &(nvsData.data), sizeof(pairingData));
			// set WiFi channel
			wifi_init_config_t cfg = esp_wifi_init_config_default();
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
			vTaskDelay(40);
		}
	}

	//-------------------------------------------------------------------------
	static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
		this_object->_espnow_send_cb(mac_addr,sendStatus);
	}
	/* ESPNOW sending or receiving callback function is called in WiFi task.
	 * Users should not do lengthy operations from this task. Instead, post
	 * necessary data to a queue and handle it from a lower priority task. */
	void _espnow_send_cb(const uint8_t *mac_addr,	esp_now_send_status_t status) {

		espnow_send_cb_t resultado;
		memcpy(resultado.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
		resultado.status = status;
		if (mac_addr == NULL) {
			ESP_LOGE(TAG4, "Send cb arg error");
			return;
		}

		if(debug) ESP_LOGI(TAG4, "ENVIO ESPNOW status: %s", (status)?"ERROR":"OK");
		if(debug) ESP_LOGI(TAG4, "MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[5],mac_addr[4],mac_addr[3],mac_addr[2],mac_addr[1],mac_addr[0]);
		if(status == 0){
			if(debug) ESP_LOGI(TAG4, " >> Exito de entrega");
			if(pairingStatus==PAIR_PAIRED && mensaje_enviado)  // será un mensaje a la pasarela, se podría comprobar la mac
			{
				mensaje_enviado = false;
				if (terminar && !esperando) gotoSleep();
			}
		}
//		else{
//			if(debug) ESP_LOGI(TAG4, " >> Error de entrega");
//			if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
//			{
//				//no hemos conseguido hablar con la pasarela emparejada...
//				// invalidamos config en flash;
//				//				memset(&nvsData, 0, sizeof(struct_rtc));
//				//				nvs_saving_task();
//				if(debug)  ESP_LOGI(TAG4, " INFO de emparejamiento invalidada");
//				pairingStatus = PAIR_REQUEST; // volvemos a intentarlo?
//				mensaje_enviado=false;
//				terminar=false;
//				//				ESP_ERROR_CHECK(esp_wifi_stop());
//				//				vTaskDelay(100 / portTICK_PERIOD_MS);
//				//				autopairing_init();
//				//				vTaskDelay(timeOut / portTICK_PERIOD_MS);
//				//				gotoSleep();
//			}
//		}
		if(xQueueSend(cola_resultado_enviados,&resultado, ESPNOW_MAXDELAY) != pdTRUE)ESP_LOGW(TAG, "Send send queue fail");
	}
	//-----------------------------------------------------------------------------
	static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
	{
		this_object->_espnow_recv_cb(recv_info,data,len);
	}
	void _espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
		uint8_t * mac_addr = recv_info->src_addr;
		uint8_t type = data[0];
		uint8_t i;
		struct_pairing *punt = (struct_pairing*) data;
		struct_espnow_rcv_msg *my_msg = (struct_espnow_rcv_msg*)malloc(sizeof(struct_espnow_rcv_msg));

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
			my_msg->topic = (char*)malloc(i - 1);
			my_msg->payload = (char*)malloc(len - i - 1);
			snprintf(my_msg->topic, i, "%s", (char*)data + 1);
			snprintf(my_msg->payload, len - i, "%s", (char*)data + i + 1);
			user_callback(my_msg);
//			mqtt_process_msg(my_msg);
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
	void set_callback( void (*_user_callback)(struct_espnow_rcv_msg*) )
	{
		user_callback=_user_callback;
	}
	//------------------------------------------------------------------------
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

	bool espnow_send_check(char * mensaje, bool fin = true, uint8_t _msgType = DATA)
	{
		esperando = true;
		timeOut += 500;
		return espnow_send(mensaje, fin, _msgType | CHECK);
	}

	//-----------------------------------------------------------
	uint8_t espnow_send(char * mensaje, bool fin = true, uint8_t _msgType = DATA)
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
	int mensajes_enviados(){ return mensajes_sent; }

	//-----------------------------------------------------------
	bool emparejado(){ return (pairingStatus==PAIR_PAIRED) ; }

	//-----------------------------------------------------------
	bool envio_disponible() { return (pairingStatus==PAIR_PAIRED && mensaje_enviado==false && terminar==false) ; }

	//--------------------------------------------------------
	void gotoSleep() {
		// get deep sleep enter time
//		gettimeofday(&rtcData.sleep_enter_time, NULL);
		// add some randomness to avoid collisions with multiple devices
		if(debug) ESP_LOGI(TAG7, "Apaga y vamonos");
		// enter deep sleep
		//	esp_deep_sleep_start();
	}
	//---------------------------------------------------------
	esp_err_t setup(void);
	//-----------------------------------------------------------

	void start_connection_task(){
		wifi_espnow_init();
		xTaskCreate(&startTaskImpl,"TASK", 4096, this_object, 1, &conexion_hand);
	}

private:

	static void startTaskImpl(void* pvParameters){
		reinterpret_cast<AUTOpairing_t*>(pvParameters)->keep_connection_task();
	}

	void keep_connection_task()
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
			{
				if(debug) ESP_LOGI(TAG2,"Pairing request on channel %d" , espnow_channel );

				if(debug) ESP_LOGI(TAG, "Pairing request on channel %u\n", espnow_channel);
				// clean esp now
				ESP_ERROR_CHECK(esp_now_deinit());
				// set WiFi channel
				uint8_t primary = -1;
				wifi_second_chan_t secondary;
				wifi_init_config_t cfg = esp_wifi_init_config_default();
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


				// set pairing data to send to the server
				pairingData.msgType = PAIRING;
				pairingData.id = ESPNOW_DEVICE;

				// send request
				esp_now_send(s_example_broadcast_mac, (uint8_t *) &pairingData, sizeof(pairingData));
				pairingStatus = PAIR_REQUESTED;
				break;
			}


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
AUTOpairing_t *AUTOpairing_t::this_object=NULL;
