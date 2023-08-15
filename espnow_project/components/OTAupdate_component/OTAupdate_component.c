
#include "OTAupdate_component.h"

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;


static const char *TAG = "* advanced https ota";


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
			ESP_LOGI(TAG, "OTA started");
			break;
		case ESP_HTTPS_OTA_CONNECTED:
			ESP_LOGI(TAG, "Connected to server");
			break;
		case ESP_HTTPS_OTA_GET_IMG_DESC:
			ESP_LOGI(TAG, "Reading Image Description");
			break;
		case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
			ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
			break;
		case ESP_HTTPS_OTA_DECRYPT_CB:
			ESP_LOGI(TAG, "Callback to decrypt function");
			break;
		case ESP_HTTPS_OTA_WRITE_FLASH:
			ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
			break;
		case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
			ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
			break;
		case ESP_HTTPS_OTA_FINISH:
			ESP_LOGI(TAG, "OTA finish");
			break;
		case ESP_HTTPS_OTA_ABORT:
			ESP_LOGI(TAG, "OTA abort");
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
		ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}

	if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
		ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
		return ESP_FAIL;
	}

    /**
     * Secure version check from firmware image header prevents subsequent download and flash write of
     * entire firmware image. However this is optional because it is also taken care in API
     * esp_https_ota_finish at the end of OTA update procedure.
     */
    const uint32_t hw_sec_version = esp_efuse_read_secure_version();
    if (new_app_info->secure_version < hw_sec_version) {
        ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed, %"PRIu32" < %"PRIu32, new_app_info->secure_version, hw_sec_version);
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
	ESP_LOGI(TAG, "Starting Advanced OTA example");
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
		ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
		esp_deep_sleep_start();
	}

	esp_app_desc_t app_desc;
	err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
		goto ota_end;
	}
	err = validate_image_header(&app_desc);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "image header verification failed");
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
		ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
	}

	if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
		// the OTA image was not completely received and user can customise the response to this situation.
		ESP_LOGE(TAG, "Complete data was not received.");
		goto ota_end;
	} else {
		ota_finish_err = esp_https_ota_finish(https_ota_handle);
		if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
			ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			esp_restart();
		} else {
			if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
				ESP_LOGE(TAG, "Image validation failed, image is corrupted");
			}
			ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
			goto ota_end;
		}
	}

	ota_end:
	esp_https_ota_abort(https_ota_handle);
	ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
	esp_deep_sleep_start();
}
