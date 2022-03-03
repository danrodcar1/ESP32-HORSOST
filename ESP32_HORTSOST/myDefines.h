/**********************************************************************
   CONFIGS.- WiFi & MQTT PROTOCOL
***********************************************************************/
//#define WIFI_SSID           "huerticawifi"
//#define WIFI_PASSWORD       "4cc3sshu3rt1c4"
#define WIFI_SSID           "HortSost"
#define WIFI_PASSWORD       "9b11c2671e5b"

#define MQTT_SERVER         "huertociencias.uma.es"
#define MQTT_PORT           8163
#define MQTT_USER           "huerta"
#define MQTT_PASSWORD       "accesohuertica"
#define MQTT_FINGERPRINT    "f6 59 59 a8 8d 75 86 07 ce a3 1a c3 93 3e 65 5f ae 72 99 45"

#define OTA_URL             "https://huertociencias.uma.es/esp8266-ota-update"
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".doitESP32devkitV1" 
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
//#define __DEBUG__ false   //set to true for debug output, false for no debug ouput
//#define Serial if(__DEBUG__)Serial

#define TYPE_NODE String("meteorologia")
/**********************************************************************
   CONFIGS.- SENSORS
***********************************************************************/
#define WATCHDOG_TIMEOUT_S 8  //time in ms to trigger the watchdog

#define ANEMOMETER_PIN 35 //digital pin 35 (interrupt 0)
#define RAINGAUGE_PIN 34 //digital pin 34 (interrupt 1)
#define WINDVANE_PIN 32 //ADC1_CH4

#define WIND_SAMPLING_SECONDS 10 //from 5 to 15 seconds would be recommended
#define WIND_SAMPLES_SIZE 6 //this value MUST be 60secs/WIND_SAMPLING_SECONDS, eg. 60/6 => 10=WIND_SAMPLES_SIZE
#define WIND_AVG_MINUTE_LOG_SIZE 18 //this MUST be equal or greater than 1 to store the current sample

/**********************************************************************
  WIND/RAIN CALIBRATION VALUES - SPEED, RAIN & WIND ANGLE
***********************************************************************/

#define ANEMOMETER_SPEED_FACTOR 2.4 //if no specs available use this setting
#define ANEMOMETER_CIRCUMFERENCE_MTS 0.4367 // circumference in meters from center cup of anemometer
#define ANEMOMETER_CYCLES_PER_LOOP 2 // tipically cup anemometers count twice on a full loop
#define RAIN_BUCKET_MM_PER_CYCLE 0.2794 //if no specs available use this setting 

// ESP32 A/D values read for each wind vane direction
#define VANE_AD_N 2950
#define VANE_AD_NE 1660 
#define VANE_AD_E 200 
#define VANE_AD_SE 560
#define VANE_AD_S 965
#define VANE_AD_SW 2330
#define VANE_AD_W 3830
#define VANE_AD_NW 3460

// ESP32 length-window filter
#define FILTER_LEN  15
