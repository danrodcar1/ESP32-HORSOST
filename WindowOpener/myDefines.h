/**********************************************************************
   CONFIGS.- WiFi & MQTT PROTOCOL
***********************************************************************/
#define WIFI_SSID           "huerticawifi"
#define WIFI_PASSWORD       "4cc3sshu3rt1c4"

#define MQTT_SERVER         "huertociencias.uma.es"
#define MQTT_PORT           8163
#define MQTT_USER           "huerta"
#define MQTT_PASSWORD       "accesohuertica"
#define MQTT_FINGERPRINT    "f6 59 59 a8 8d 75 86 07 ce a3 1a c3 93 3e 65 5f ae 72 99 45"

#define OTA_URL             "https://huertociencias.uma.es/esp8266-ota-update"
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".esp32" 
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
//#define __DEBUG__ true   //set to true for debug output, false for no debug ouput
//#define Serial if(__DEBUG__)Serial

#define TYPE_NODE String("ghWindow")
/**********************************************************************
   CONFIGS.- SENSORS
***********************************************************************/
#define WATCHDOG_TIMEOUT_S 8  //time in ms to trigger the watchdog

#define CHECK_UPDATE_TIMER 10L //send sensor data each 10'
#define WIND_SAMPLING_SECONDS 10 //from 5 to 15 seconds would be recommended
#define WIND_SAMPLES_SIZE 6 //this value MUST be 60secs/WIND_SAMPLING_SECONDS, eg. 60/6 => 10=WIND_SAMPLES_SIZE
#define WIND_AVG_MINUTE_LOG_SIZE 18 //this MUST be equal or greater than 1 to store the current sample

// ESP32 length-window filter
#define FILTER_LEN  15
