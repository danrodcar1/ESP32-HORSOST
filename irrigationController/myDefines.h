/**********************************************************************
   CONFIGS.- WiFi & MQTT PROTOCOL
***********************************************************************/
#define WIFI_SSID           "huerticawifi"
#define WIFI_PASSWORD       "4cc3sshu3rt1c4"

#define MQTT_SERVER         "huertociencias.uma.es"
#define MQTT_PORT           8163
#define MQTT_USER           "huerta"
#define MQTT_PASSWORD       "accesohuertica"


#define OTA_URL             "https://huertociencias.uma.es/esp8266-ota-update"
#define OTA_FINGERPRINT     "fd416a4a7451f42849ace38b768a6a0cf8918b86" // sustituir valor
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
//#define __DEBUG__ true   //set to true for debug output, false for no debug ouput
//#define Serial if(__DEBUG__)Serial

#define TYPE_NODE String("irrigation")

#define WATCHDOG_TIMEOUT_S 8  //time in ms to trigger the watchdog

#define SERIAL_BAUD_RATE    115200

#define CONTROLLER_PIN_1 5 //D1 
#define CONTROLLER_PIN_2 4 //D2
#define CONTROLLER1_LED 12
#define CONTROLLER2_LED 14

#define CHECK_UPDATE_TIMER 10L 
#define SEND_MSG_MINUTES 10L
