#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "myDefines.h"
#include "esp_adc_cal.h"
#include <SimpleTimer.h> //repetitive tasks
#include "esp_system.h" //watchdog
#include "time.h"
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include "build_defs.h"

/**********************************************************************
   VARS
***********************************************************************/

// ESP32  WiFi & UPDATE SERVER
WiFiClientSecure espClient;
PubSubClient client(espClient);

const unsigned char FWVER[] =
{
  VERSION_MAJOR_INIT,
  '.',
  VERSION_MINOR_INIT,
  '-', 'V', '-',
  BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
  '-',
  BUILD_MONTH_CH0, BUILD_MONTH_CH1,
  '-',
  BUILD_DAY_CH0, BUILD_DAY_CH1,
  'T',
  BUILD_HOUR_CH0, BUILD_HOUR_CH1,
  ':',
  BUILD_MIN_CH0, BUILD_MIN_CH1,
  ':',
  BUILD_SEC_CH0, BUILD_SEC_CH1,
  '\0'
};

hw_timer_t * watchDogTimer = NULL; //watchdog timer declaration
SimpleTimer timerManager; //timer declaration

volatile unsigned long nextTimeAnemometerInterrupt = 0 , nextTimeRainIterrupt = 0  ; //software debounce variables
volatile int anemometerCyclesCounter = 0, anemometerMinuteCyclesCounter = 0, rainCyclesCounter = 0; //interrupt counters
unsigned long lastMinuteSampleMillis = 0;

int windvaneADRefValues[] = { VANE_AD_N, VANE_AD_NE, VANE_AD_E, VANE_AD_SE, VANE_AD_S, VANE_AD_SW, VANE_AD_W, VANE_AD_NW };

struct WindSample {
  float windCyclesPerSecond;
  int windAngle;
  unsigned long sampleMillis;
};

struct SensorsSample {
  float windCyclesPerSecond;
  int windAngle;
  float gustCyclesPerSecond;
  int gustAngle;
  int rainCyclesPerMinute;
  int windCyclesPerMinute;
  unsigned long  elapsedSeconds;
  unsigned long  sampleMillis;
};

String maintenanceTopic;
bool maintenanceState = false;

SensorsSample avgMinuteSamplesLog[WIND_AVG_MINUTE_LOG_SIZE];
WindSample windSamples[WIND_SAMPLES_SIZE];

// ADC FILTER VARS
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Filtered = 0;

void IRAM_ATTR watchDogInterrupt();
void watchDogRefresh();

unsigned long previousMillis;
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  connectToNetwork();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);
  checkForUpdates();

  //init watchdog
  watchDogTimer = timerBegin(0, 80, true); //timer 0, div80
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);

  initSamplesArrays();

  pinMode(ANEMOMETER_PIN, INPUT);
  pinMode(RAINGAUGE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countAnemometerCycles, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAINGAUGE_PIN), countRainCycles, FALLING);

  timerManager.setInterval(WIND_SAMPLING_SECONDS * 1000, captureAndSendPartialSample); // Capture values each 10''
  timerManager.setInterval(DATA_SAMPLING_MINUTES * 60000L, captureAndSendMinuteSample); // Send values each 10'
}

void loop()
{
  unsigned long currentMillis = millis();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if ((maintenanceState == true) && ((unsigned long)(currentMillis - previousMillis) >= MAINTENANCE_MAX_MINUTES * 60000L)) {
    maintenanceState = false;
    previousMillis = millis();
  }
  timerManager.run();
  watchDogRefresh();
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToNetwork() {
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
  espClient.setInsecure();
  espClient.setTimeout(12);
}

void reconnect() {
  String clientId = "ESP32Client-";
  uint32_t chipID = ESP.getEfuseMac();
  clientId += String(chipID);
  String topic_string = ("orchard/" + TYPE_NODE + "/" + clientId + "/connection");                             // Select topic by ESP ID
  const char* conexion_topic = topic_string.c_str();

  String topic_string_sub = ("orchard/" + TYPE_NODE + "/" + clientId + "/mantenimiento");     // Select topic by ESP ID
  maintenanceTopic = topic_string_sub;
  const char* subTopic = topic_string_sub.c_str();
  // BUCLE DE CHECKING DE CONEXIÓN AL SERVICIO MQTT
  while (!client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");
    Serial.print("Conectando a ");
    Serial.print(MQTT_SERVER);
    Serial.print(" como ");
    Serial.println(clientId);

    // Intentando conectar
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, conexion_topic, 2, true, "Offline", true)) {
      Serial.println("conectado");
      // Nos suscribimos a los siguientes topics
      client.subscribe(subTopic);
      Serial.printf("\r\Subscribed to:\t%s", subTopic);
      Serial.println();
      // Publicamos el estado de la conexion en el topic
      client.publish(conexion_topic, "Online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Espera 5s antes de reintentar conexión
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Command from MQTT broker is : [");
  Serial.print(topic);
  Serial.println("] ");
  char message_buff[100];
  if (strcmp(topic, maintenanceTopic.c_str()) == 0)
  {
    int i;
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    String msgString = String(message_buff);
    if (msgString.equals("true")){
      maintenanceState = true;
      previousMillis = millis();
    }
    if (msgString.equals("false")) maintenanceState = false;
  }
}

void captureAndSendPartialSample() {

  shiftArrayToRight_ws(windSamples, WIND_SAMPLES_SIZE);
  float prevSampleMillis = windSamples[1].sampleMillis;
  unsigned long currentSampleMillis = millis();
  float elapsedSeconds = (float)(currentSampleMillis - prevSampleMillis) / (float)1000;
  float windCyclesPerSecond = (float)anemometerCyclesCounter / elapsedSeconds;
  float anemometerCyclesCounterRegister = anemometerCyclesCounter;
  anemometerCyclesCounter = 0;//reset the partial interrupt counter to 0
  int windAngle = analogToAngleDirection(readADC_Avg(analogRead(WINDVANE_PIN)), windvaneADRefValues);
  windSamples[0] = {windCyclesPerSecond, windAngle, currentSampleMillis};
}

void captureAndSendMinuteSample() {
  String clientId = "ESP32Client-";
  uint32_t chipID = ESP.getEfuseMac();
  clientId += String(chipID);
  String topic_string = ("orchard/" + TYPE_NODE + "/" + clientId + "/data");                             // Select topic by ESP ID
  const char* fullSample_topic = topic_string.c_str();

  float prevSampleMillis = lastMinuteSampleMillis;
  unsigned long currentSampleMillis = millis();
  float elapsedSeconds = (float)(currentSampleMillis - prevSampleMillis) / (float)1000;
  float windCyclesPerSecond = (float)anemometerMinuteCyclesCounter / elapsedSeconds;
  int avgWindAngle = 0, gustAngle = 0;
  float gustCyclesPerSecond = 0;

  calcValuesFromWindSamples(windSamples, WIND_SAMPLES_SIZE, avgWindAngle, gustCyclesPerSecond, gustAngle);

  SensorsSample avgMinuteSample = {
    windCyclesPerSecond,
    avgWindAngle,
    gustCyclesPerSecond,
    gustAngle,
    rainCyclesCounter,
    anemometerMinuteCyclesCounter,
    elapsedSeconds,
    currentSampleMillis
  };

  rainCyclesCounter = 0; //reset the interrupt counter to 0 each 10' (more accurate)
  anemometerMinuteCyclesCounter = 0;//reset the interrupt counter to 0
  lastMinuteSampleMillis = currentSampleMillis;
  shiftArrayToRight_ss(avgMinuteSamplesLog, WIND_AVG_MINUTE_LOG_SIZE);
  avgMinuteSamplesLog[0] = avgMinuteSample;

  client.publish(fullSample_topic, sendFullSamples(avgMinuteSamplesLog, 1).c_str(), true);
  checkForUpdates();
}

String sendFullSamples(SensorsSample * samples, int samplesToSend) {
  for (int i = 0; i < samplesToSend; i++) {
    DynamicJsonDocument jsonRoot(2048);
    String jsonString;
    JsonObject Counter1 = jsonRoot.createNestedObject("Counter1");
    Counter1["SensorLluvia"] = samples[i].rainCyclesPerMinute;
    Counter1["SensorViento"] = samples[i].windCyclesPerMinute;

    //    jsonRoot["velocidad"] = (float)samples[i].windCyclesPerMinute * (float)ANEMOMETER_SPEED_FACTOR / (float)samples[i].elapsedSeconds;
    jsonRoot["velocidad"] = (samples[i].windCyclesPerSecond / (float)ANEMOMETER_CYCLES_PER_LOOP * (float)ANEMOMETER_CIRCUMFERENCE_MTS * (float)ANEMOMETER_SPEED_FACTOR) * 3.6;
    jsonRoot["velocidadRafaga"] = (samples[i].gustCyclesPerSecond / (float)ANEMOMETER_CYCLES_PER_LOOP * (float)ANEMOMETER_CIRCUMFERENCE_MTS * (float)ANEMOMETER_SPEED_FACTOR) * 3.6;
    jsonRoot["direccion"] = samples[i].windAngle;
    jsonRoot["direccionRafaga"] = samples[i].gustAngle;
    jsonRoot["litros"] = samples[i].rainCyclesPerMinute * (float)RAIN_BUCKET_MM_PER_CYCLE;
    jsonRoot["segundos"] = samples[i].elapsedSeconds;
    serializeJson(jsonRoot, jsonString);
    return jsonString;
  }
}

void initSamplesArrays() {
  for (int i = 0; i < WIND_SAMPLES_SIZE; i++) {
    windSamples[i] = {0};
  }
  for (int c = 0; c < WIND_AVG_MINUTE_LOG_SIZE; c++) {
    avgMinuteSamplesLog[c] = {0};
  }
}

uint32_t readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;

  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if (AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for (i = 0; i < FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum / FILTER_LEN);
}

/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/

void IRAM_ATTR watchDogInterrupt() {
  ets_printf("reboot\n");
  esp_restart();
}

void watchDogRefresh()
{
  timerWrite(watchDogTimer, 0); //reset timer (feed watchdog)
}

void countRainCycles() {
  if (maintenanceState == false) {
    if (nextTimeRainIterrupt == 0 || nextTimeRainIterrupt < millis()) {
      rainCyclesCounter++;
      nextTimeRainIterrupt = millis() + 100;
    }
  }
}

void countAnemometerCycles() {
  if (nextTimeAnemometerInterrupt == 0 || nextTimeAnemometerInterrupt < millis()) {
    anemometerCyclesCounter++;
    anemometerMinuteCyclesCounter++;
    nextTimeAnemometerInterrupt = millis() + 10;
  }
}

/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/

void calcValuesFromWindSamples(WindSample * ws, int windSamplesSize, int & avgWindAngle, float & gustCyclesPerSecond, int & gustAngle) {
  int tempAngles[windSamplesSize] = {0};
  for (int i = 0; i < windSamplesSize; i++) {
    if (ws[i].windCyclesPerSecond >= gustCyclesPerSecond) {
      gustCyclesPerSecond = ws[i].windCyclesPerSecond;
      gustAngle = ws[i].windAngle;
    }
    tempAngles[i] = ws[i].windAngle;
  }
  avgWindAngle = meanAngle(tempAngles, windSamplesSize);
}

/**
  Shift array to the right.
  Element on index 0 goes to 1, 1 to 2,..., latest one goes out,
  First element (index 0) is reseted to 0
*/
void shiftArrayToRight_ws(WindSample* arrayToShift, int arraySize) {
  for (int i = arraySize - 1; i > 0; i--) {
    arrayToShift[i] = arrayToShift[i - 1];
  }
  arrayToShift[0] = {0};
}

/**
  Shift array to the right.
  Element on index 0 goes to 1, 1 to 2,..., latest one goes out,
  First element (index 0) is reseted to 0
*/
void shiftArrayToRight_ss(SensorsSample* arrayToShift, int arraySize) {
  for (int i = arraySize - 1; i > 0; i--) {
    arrayToShift[i] = arrayToShift[i - 1];
  }
  arrayToShift[0] = {0};
}

/**
  return the angle closest to the InputValue reference
*/
int analogToAngleDirection(int adInputValue, int * referenceValues) {
  int angles[] = {
    0/*N*/, 45/*NE*/, 90/*E*/, 135/*SE*/, 180/*S*/, 225/*SW*/, 270/*W*/, 315/*NW*/
  };
  int lowerDiff = 0, angle = 0, i = 0;
  for (i = 0; i < 8; i++) {
    int tempDiff = adInputValue - referenceValues[i];
    tempDiff = abs(tempDiff);
    if (i == 0 || tempDiff < lowerDiff) {
      lowerDiff = tempDiff;
      angle = angles[i];
    }
  }
  return angle;
}



/**
  function to average angles using its cos and sin components,
  (averaging angles by its nominal value (180º, 350º, etc) wouldn't result in a meaningfull value
*/
int meanAngle (int *angles, int size)
{
  double yPart = 0, xPart = 0;
  int i;

  for (i = 0; i < size; i++)
  {
    double angle = (double)angles[i];
    xPart += cos (angle * M_PI / 180);
    yPart += sin (angle * M_PI / 180);
  }
  //if y and x is 0, then the atan2 is undefined, so we return the first angle as average result
  if (yPart == 0 and xPart == 0) {
    return angles[0];
  }
  else {
    double avgAngle =  atan2 (yPart / size, xPart / size) * 180 / M_PI;
    if (avgAngle < 0)
      return (int)(avgAngle + 360);
    else
      return (int)avgAngle;
  }
}

void checkForUpdates() {
  Serial.println("Check FOTA...");
  switch (httpUpdate.update(espClient, OTA_URL, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(" HTTP update failed: Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F(" El dispositivo ya está actualizado"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F(" OK"));
      break;
  }
}
