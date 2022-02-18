#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "myDefines.h"
#include "esp_adc_cal.h"
#include <SimpleTimer.h> //repetitive tasks
#include "esp_system.h" //watchdog
#include "time.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>


/**********************************************************************
   VARS
***********************************************************************/

// ESP32  WiFi & UPDATE SERVER
WiFiClient espClient;
PubSubClient client(espClient);
AsyncWebServer server(80);
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

char espID[32];
String ESP32_ID;

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
  unsigned long  sampleMillis;
};

SensorsSample avgMinuteSamplesLog[WIND_AVG_MINUTE_LOG_SIZE];
WindSample windSamples[WIND_SAMPLES_SIZE];

// ADC FILTER VARS
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Filtered = 0;

void IRAM_ATTR watchDogInterrupt();
void watchDogRefresh();

/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200); // Inicializamos el puerto serial
  connectToNetwork();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  //init watchdog
  watchDogTimer = timerBegin(0, 80, true); //timer 0, div80
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);

  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(gmtOffset_sec);


  initSamplesArrays();

  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  pinMode(RAINGAUGE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countAnemometerCycles, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAINGAUGE_PIN), countRainCycles, FALLING);

  timerManager.setInterval(WIND_SAMPLING_SECONDS * 1000, captureAndSendPartialSample); // Capture values each 10''
  timerManager.setInterval(10L * 60000L, captureAndSendMinuteSample); // Send values each 10'
}

void loop()
{
  //readLocalTime();
  timerManager.run();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  AsyncElegantOTA.loop();
  watchDogRefresh();
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/

void readLocalTime()
{
  while (!timeClient.update()) {
    Serial.println("Failed to obtain time");
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  // Extract time
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  if (timeStamp == "00:00:00") {
    rainCyclesCounter = 0; //reset the interrupt counter to 0 AT MIDNIGHT
  }
}

void captureAndSendPartialSample() {
  shiftArrayToRight_ws(windSamples, WIND_SAMPLES_SIZE);
  float prevSampleMillis = windSamples[1].sampleMillis;
  unsigned long currentSampleMillis = millis();
  float elapsedSeconds = (float)(currentSampleMillis - prevSampleMillis) / (float)1000;
  float windCyclesPerSecond = (float)anemometerCyclesCounter / elapsedSeconds;
  anemometerCyclesCounter = 0;//reset the partial interrupt counter to 0
  int windAngle = analogToAngleDirection(readADC_Avg(analogRead(WINDVANE_PIN)), windvaneADRefValues);
  windSamples[0] = {windCyclesPerSecond, windAngle, currentSampleMillis};
  Serial.println(sendWindPartialSample(windSamples[0]).c_str());
}

String sendWindPartialSample(WindSample ws) {
  DynamicJsonDocument jsonRoot(2048);
  String jsonString;
  JsonObject partialSample = jsonRoot.createNestedObject("partialSample");
  partialSample["windSpeed"] = truncar(ws.windCyclesPerSecond / (float)ANEMOMETER_CYCLES_PER_LOOP * (float)ANEMOMETER_CIRCUMFERENCE_MTS * (float)ANEMOMETER_SPEED_FACTOR, 3);
  partialSample["windAngle"] = ws.windAngle;
  partialSample["sampleTime"] = ws.sampleMillis;
  serializeJson(jsonRoot, jsonString);
  return jsonString;
}

String sendFullSamples(SensorsSample * samples, int samplesToSend) {
  for (int i = 0; i < samplesToSend; i++) {
    DynamicJsonDocument jsonRoot(2048);
    String jsonString;
    JsonObject fullSample = jsonRoot.createNestedObject("fullSample");
    fullSample["windSpeed"] = truncar(samples[i].windCyclesPerSecond / (float)ANEMOMETER_CYCLES_PER_LOOP * (float)ANEMOMETER_CIRCUMFERENCE_MTS * (float)ANEMOMETER_SPEED_FACTOR, 3);
    fullSample["windAngle"] = samples[i].windAngle;
    fullSample["gustWind"] = truncar(samples[i].gustCyclesPerSecond / (float)ANEMOMETER_CYCLES_PER_LOOP * (float)ANEMOMETER_CIRCUMFERENCE_MTS * (float)ANEMOMETER_SPEED_FACTOR, 3);
    fullSample["gustWindAngle"] = samples[i].gustAngle;
    fullSample["rmm"] = samples[i].rainCyclesPerMinute * (float)RAIN_BUCKET_MM_PER_CYCLE;
    fullSample["sampleTime"] = samples[i].sampleMillis;
    serializeJson(jsonRoot, jsonString);
    return jsonString;
  }
}

void captureAndSendMinuteSample() {
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
    currentSampleMillis
  };

  rainCyclesCounter = 0; //reset the interrupt counter to 0 each 10' (more accurate)
  anemometerMinuteCyclesCounter = 0;//reset the interrupt counter to 0
  lastMinuteSampleMillis = currentSampleMillis;
  shiftArrayToRight_ss(avgMinuteSamplesLog, WIND_AVG_MINUTE_LOG_SIZE);
  avgMinuteSamplesLog[0] = avgMinuteSample;
  Serial.println(sendFullSamples(avgMinuteSamplesLog, 1).c_str());
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



void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(wifi_ssid, wifi_password);
}

void connectToNetwork() {
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
  startUpdateServer();
}

void reconnect() {
  // BUCLE DE CHECKING DE CONEXIÓN AL SERVICIO MQTT
  while (!client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");

    // Generación del nombre del cliente en función de la dirección MAC y los ultimos 8 bits del contador temporal
    String clientId = "ESP32Client-";
    uint32_t chipID = ESP.getEfuseMac();
    clientId += String(chipID);
    ESP32_ID = clientId;
    clientId.toCharArray(espID, 32);
    Serial.print("Conectando a ");
    Serial.print(MQTT_SERVER);
    Serial.print(" como ");
    Serial.println(clientId);

    // Intentando conectar
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, conexion_topic, 2, true, "Offline", true)) {
      Serial.println("conectado");
      // Nos suscribimos a los siguientes topics

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
  client.publish(conexion_topic, "Online", true);
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Command from MQTT broker is : [");
  Serial.print(topic);
  Serial.println("] ");
  char message_buff[100];
}

String EspDataWifiSerialize (void)
{
  DynamicJsonDocument jsonRoot(2048);
  String jsonString;

  jsonRoot["ChipID"] = espID;
  //jsonRoot["Uptime"] = TiempoPrograma;
  JsonObject Wifi = jsonRoot.createNestedObject("WiFi");
  Wifi["SSID"] = wifi_ssid;
  char buffer_ip[] = "xxx.xxx.xxx.xxx";
  IPAddress wifi_ip = WiFi.localIP();
  wifi_ip.toString().toCharArray(buffer_ip, 16);
  Wifi["IP"] = buffer_ip;
  Wifi["RSSI"] = WiFi.RSSI();

  serializeJson(jsonRoot, jsonString);
  return jsonString;
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
  if (nextTimeRainIterrupt == 0 || nextTimeRainIterrupt < millis()) {
    rainCyclesCounter++;
    nextTimeRainIterrupt = millis() + 100;
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

float truncar (float num, int pos) {
  String string = String(num);
  unsigned int longitud = string.length();
  float decimalLength = string.indexOf('.') + longitud;
  String numStr = string.substring(0, decimalLength + pos);
  return numStr.toFloat();
}

void startUpdateServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}
