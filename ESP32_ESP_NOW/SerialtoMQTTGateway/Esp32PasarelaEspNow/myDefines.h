/**********************************************************************
   CONFIGS.- WiFi & MQTT PROTOCOL
***********************************************************************/
#define WIFI_SSID           "huerticawifi"
#define WIFI_PASSWORD       "4cc3sshu3rt1c4"
#define WIFI_CHANNEL 0

#define OTA_URL             "https://huertociencias.uma.es/esp8266-ota-update"
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".esp32" 
#define VERSION_MAJOR 1
#define VERSION_MINOR 0

#define LED_STATUS 16  
#define SERIAL_BAUD_RATE 115200

#define CHECK_UPDATE_TIMER 10L 
#define WATCHDOG_TIMEOUT_S 8  //time in ms to trigger the watchdog
