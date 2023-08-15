
set(SOURCES "OTAupdate_component.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES 
                    	"efuse"
                    	"nvs_flash"
                    	"esp_wifi"
                    	"esp_timer"
                    	"esp_http_client" 
                    	"esp_https_ota" # HTTPS OTA
                    	"app_update"	# OTA
                    )

