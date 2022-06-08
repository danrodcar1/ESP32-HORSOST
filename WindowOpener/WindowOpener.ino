#include <ArduinoJson.h>
#include "Button2.h"
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
//WiFiClientSecure espClient;
WiFiClient espClient;

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

// Stop-end
const int END_SW_PIN[] = {34, 35};
int lengthSW = sizeof(END_SW_PIN) / sizeof(END_SW_PIN[0]);

// Operational button
const int BUTTON_PIN = 39;
Button2 button;

// Cooling system
const int FAN_PWR_PIN = 19;
const int LEDC_BASE_FREQ_30K = 30000;
const int LEDC_TIMER_12_BIT = 12;
const int LEDC_CHANNEL_2 = 2;

// Driver TB6612FNG.- NOTE: has maximum PWM switching frequency of 100kHz.
const int pinPWMA = 13;
const int pinAIN2 = 12;
const int pinAIN1 = 14;
const int pinBIN1 = 26;
const int pinBIN2 = 25;
const int pinPWMB = 33;
const int pinSTBY = 27;

const int LEDC_BASE_FREQ_20K = 20000;
const int LEDC_TIMER_10_BIT = 10;
const int LEDC_CHANNEL_0 = 0;
const int LEDC_CHANNEL_1 = 1;

const int pinMotorA[3] = { pinPWMA, pinAIN1, pinAIN2 };
const int pinMotorB[3] = { pinPWMB, pinBIN1, pinBIN2 };
int lengthMotor = sizeof(pinMotorB) / sizeof(pinMotorB[0]);

enum windowState {
  ACTIVATE_RIGHT_WINDOW,
  ACTIVATE_LEFT_WINDOW,
  ACTIVATE_BOTH_WINDOW
}; windowState winCMD = ACTIVATE_RIGHT_WINDOW;

struct __attribute__((packed)) MQTT_MSG {
  windowState winCMD;
  int numRelay;
  int cmdRelay;
  const char* id;
} messageReceived;
// System calibration.- rack-pinion mechanism
/*
   Note: In order to avoid overheating in the motor driver,
   the duty cycle will be halved and the new rpm value will be averaged
*/
const int zPinion = 16; // number of pinion teeth (teeth)
const float nRack = 3.19; // number of rack teeth per centimeter (teeth/cm)
const float tsPinion = 22.5; //pinion turning speed (rpm) at full PWM => 45rpm

int maxTimeOpenA = 0;
int maxTimeOpenB = 0;
float motionFactor = 0;
int movCMD = 0; // 0 = STOP; 1 = FORWARD; 2 = BACKWARD
int openPerc = 0;
int openPerc_diff = 0;
int timeTravelTarget = 0;
int travelCounter = 0;
unsigned long timeTravelStop = 0;
int openPerc_ant = 0;
int fullSpeed = 512;  // full duty long range movement (0-1024)..
int softSpeed = 256;  // mid duty for aprox movement (0-1024)..

// LM35
const int LM35_PIN = 36;
float targetTemp = 25.0;
const float errorTemp = 0.5;
int LM35_Raw_Sensor = 0;
int LM35_Filtered_Sensor = 0;
float LM35_TempC_Sensor1 = 0.0;
float Voltage = 0.0;

// ADC FILTER VARS
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;

// Other stuff
unsigned long CHECK_TEMP_PERIOD = 500;
String topic_string_sub = ("orchard/" + TYPE_NODE + "/");


void IRAM_ATTR watchDogInterrupt();
void watchDogRefresh();
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  startingIO();
  connectToNetwork();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);

  checkForUpdates();
  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, checkForUpdates); // Look for update each 10'
  timerManager.setInterval(CHECK_TEMP_PERIOD, ctrlTempFan);

  //init watchdog
  watchDogTimer = timerBegin(0, 80, true); //timer 0, div80
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);

  if (SET_WINDOW_CALIBRATION) {
    maxTimeOpenA = windowCalib(pinMotorA, END_SW_PIN[0], LEDC_CHANNEL_0, fullSpeed);
    maxTimeOpenB = windowCalib(pinMotorB, END_SW_PIN[1], LEDC_CHANNEL_1, fullSpeed);
    motionFactor = ((maxTimeOpenA + maxTimeOpenB) / 2) / 100;
  }

  // Setting default options for the green-house window
  motorInitialize();
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  button.loop();
  ctrlTempFan();

  if (winCMD == ACTIVATE_RIGHT_WINDOW) {
    windowCtrl(pinMotorA, END_SW_PIN[0], LEDC_CHANNEL_0, fullSpeed);
  }
  if (winCMD == ACTIVATE_LEFT_WINDOW) {
    windowCtrl(pinMotorB, END_SW_PIN[1], LEDC_CHANNEL_1, fullSpeed);
  }
  if (winCMD == ACTIVATE_BOTH_WINDOW) {
    windowCtrl(pinMotorA, END_SW_PIN[0], LEDC_CHANNEL_0, fullSpeed);
    windowCtrl(pinMotorB, END_SW_PIN[1], LEDC_CHANNEL_1, fullSpeed);
  }


  timerManager.run();
  watchDogRefresh();
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/

int windowCalib(const int pinMotor[3], int swStatus, uint8_t channel, int movSpeed) {
  float revDist = (float)zPinion / nRack; // For each complete revolution of the pinion, the rack will move forward as many teeth as the pinion has (cm)
  float velMove = (float)tsPinion * revDist / 60000L; // forward/reverse speed (cm/ms)
  int timeMove = 0;
  int maxTimeOpen = 0;
  float traveledSpace = 0;
  char dataBuffer[40];
  enableMotors();
  closeWindow(pinMotor, swStatus, channel, movSpeed);
  Serial.println("====== Window calibration ======");
  Serial.print((String)"Traveled Space (cm)" + '\t' + "Time (ms)" + '\n');
  for (timeMove = 0; timeMove < 20000; timeMove++) {
    moveMotorForward(pinMotor, channel, movSpeed);
    traveledSpace = (float)velMove * timeMove;//* movSpeed / fullSpeed;
    sprintf(dataBuffer, "%.3f\t%d\n", traveledSpace, timeMove);
    Serial.print(dataBuffer);
    delay (1);
    ctrlTempFan();
    watchDogRefresh();
    if (traveledSpace >= 27)movSpeed = softSpeed;
    if (traveledSpace >= 31)break;
  }
  maxTimeOpen = timeMove;
  Serial.println((String)"Opening time: " + maxTimeOpen + " ms");
  closeWindow(pinMotor, swStatus, channel, movSpeed + 256);
  disableMotors();
  stopMotor(pinMotor, channel);
  return maxTimeOpen;
}

void motorInitialize() {
  enableMotors();
  closeWindow(pinMotorA, END_SW_PIN[0], LEDC_CHANNEL_0, fullSpeed);
  closeWindow(pinMotorB, END_SW_PIN[1], LEDC_CHANNEL_1, fullSpeed);
  fullStop();
}

void windowCtrl(const int pinMotor[3], int swStatus, uint8_t channel, int movSpeed) {
  unsigned long currentMillis = millis();
  if (travelCounter <= 6) {
    if (movCMD == 0) {
      disableMotors();
      stopMotor(pinMotor, channel);
    }
    if (movCMD == 1) {
      enableMotors();
      moveMotorForward(pinMotor, channel, movSpeed);
      if (currentMillis >= timeTravelStop) {
        movCMD = 0;
        travelCounter++;
      }
    }
    if (movCMD == 2) {
      enableMotors();
      moveMotorBackward(pinMotor, channel, movSpeed);
      if (currentMillis >= timeTravelStop || digitalRead(swStatus) == 0) {
        movCMD = 0;
        travelCounter++;
      }
    }
  }
  else {
    enableMotors();
    moveMotorBackward(pinMotor, channel, movSpeed);
    if (digitalRead(swStatus) == 0) {
      openPerc_ant = 0;
      openPerc_diff = abs(openPerc - openPerc_ant);
      timeTravelTarget = (int)openPerc_diff * motionFactor; // Travel time in ms to open the window a certain %
      timeTravelStop = millis() + timeTravelTarget;
      travelCounter = 0;
      movCMD = 1;
    }
  }
}

int closeWindow(const int pinMotor[3], int swStatus, uint8_t channel, int movSpeed) {
  int swFlag;
  if (digitalRead(swStatus) == 1) {
    Serial.println("Window opened... Closing");
    for (int t = 0; t < 30000; t++) {
      moveMotorBackward(pinMotor, channel, movSpeed);
      delay (1);
      ctrlTempFan();
      watchDogRefresh();
      if (digitalRead(swStatus) == 0) break;
    }
    stopMotor(pinMotor, channel);
    swFlag = 0; //Window was closing (it was open before)
  }
  else if (digitalRead(swStatus) == 0) {
    Serial.println("Window already closed");
    swFlag = 0; //Window was already closed
  }
  Serial.println("Window closed");
  return swFlag;
}

void startingIO() {
  // sets the pins as inputs/outputs:
  // Button
  button.begin(BUTTON_PIN);
  button.setClickHandler(buttonFunction);
  button.setLongClickHandler(buttonFunction);
  button.setDoubleClickHandler(buttonFunction);
  button.setTripleClickHandler(buttonFunction);
  // Stop-end
  for (int i = 0; i < lengthSW; i++)
  {
    pinMode(END_SW_PIN[i], INPUT);
  }
  // Motor System
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ_20K, LEDC_TIMER_10_BIT);
  ledcAttachPin(pinMotorA[0], LEDC_CHANNEL_0);
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ_20K, LEDC_TIMER_10_BIT);
  ledcAttachPin(pinMotorB[0], LEDC_CHANNEL_1);
  for (int i = 1; i < lengthMotor; i++)
  {
    pinMode(pinMotorA[i], OUTPUT);
    pinMode(pinMotorB[i], OUTPUT);
    digitalWrite(pinMotorA[i], LOW);
    digitalWrite(pinMotorB[i], LOW);
  }
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, LOW); // standby on
  // Cooling system
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ_30K, LEDC_TIMER_12_BIT);
  ledcAttachPin(FAN_PWR_PIN, LEDC_CHANNEL_2);
}

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
  //  espClient.setInsecure();
  //  espClient.setTimeout(12);
}

void reconnect() {
  String clientId = "ESP32Client-";
  uint32_t chipID = ESP.getEfuseMac();
  clientId += String(chipID);
  String topic_string = ("orchard/" + TYPE_NODE + "/" + clientId + "/connection");                             // Select topic by ESP ID
  const char* conexion_topic = topic_string.c_str();

  topic_string_sub += (clientId + "/activate");
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
  StaticJsonDocument<256> doc;
  Serial.print("Command from MQTT broker is : [");
  Serial.print(topic);
  Serial.println("] ");
  char message_buff[100];
  if (strcmp(topic, topic_string_sub.c_str()) == 0)
  {
    int i;
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    DeserializationError error = deserializeJson(doc, message_buff);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
  }
}

void ctrlTempFan() {
  float currentTemperature = readTemp();
  if ((currentTemperature > targetTemp) && (abs(currentTemperature - targetTemp) >= errorTemp)) {
    ledcWrite(LEDC_CHANNEL_2, 4096);   // Turn-on fan
  }
  if ((currentTemperature <= targetTemp) && (abs(currentTemperature - targetTemp) >= errorTemp)) {
    ledcWrite(LEDC_CHANNEL_2, 1024);   // Turn-off fan
  }
}

/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/

void buttonFunction(Button2& btn) {
  switch (btn.getType()) {
    case single_click:
      openPerc = 25;
      break;
    case double_click:
      openPerc = 50;
      break;
    case triple_click:
      openPerc = 75;
      break;
    case long_click:
      Serial.println("==== Closing window ====");
      openPerc = 0;
      break;
  }
  if (openPerc > openPerc_ant) {
    Serial.print((String)"Opening windows to " + openPerc + "%" + '\n');
    movCMD = 1;
  }
  else if (openPerc < openPerc_ant) {
    Serial.print((String)"Closing windows to " + openPerc + "%" + '\n');
    movCMD = 2;
  }
  else if (openPerc == openPerc_ant) {
    Serial.print((String)"Window already open at " + openPerc + "%" + '\n');
    movCMD = 0;
  }
  openPerc_diff = abs(openPerc - openPerc_ant);
  openPerc_ant = openPerc;
  timeTravelTarget = (int)openPerc_diff * motionFactor; // Travel time in ms to open the window a certain %
  timeTravelStop = millis() + timeTravelTarget;
  Serial.print((String)"Travel time: " + timeTravelTarget + "ms" + '\n' + "Stop time: " + timeTravelStop + "ms" + '\n');
}

void IRAM_ATTR watchDogInterrupt() {
  ets_printf("reboot\n");
  esp_restart();
}

void watchDogRefresh()
{
  timerWrite(watchDogTimer, 0); //reset timer (feed watchdog)
}

/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/

float readTemp() {
  // Read LM35_Sensor ADC Pin & filter the raw signal
  LM35_Raw_Sensor = analogRead(LM35_PIN);
  LM35_Filtered_Sensor = readADC_Avg(LM35_Raw_Sensor);
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Filtered_Sensor);
  // TempC = Voltage(mV) / 10
  return (Voltage / 10);
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

uint32_t readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if (AN_Pot1_i == FILTER_LEN)AN_Pot1_i = 0;
  for (i = 0; i < FILTER_LEN; i++)Sum += AN_Pot1_Buffer[i];
  return (Sum / FILTER_LEN);
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

void moveMotorForward(const int pinMotor[3], uint8_t channel, int speed)
{
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);
  ledcWrite(channel, speed);
}

void moveMotorBackward(const int pinMotor[3], uint8_t channel, int speed)
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], HIGH);
  ledcWrite(channel, speed);
}

void fullStop()
{
  disableMotors();
  stopMotor(pinMotorA, LEDC_CHANNEL_0);
  stopMotor(pinMotorB, LEDC_CHANNEL_1);
}

void stopMotor(const int pinMotor[3], uint8_t channel)
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);
  ledcWrite(channel, 0);
}

void enableMotors()
{
  digitalWrite(pinSTBY, HIGH);
}

void disableMotors()
{
  digitalWrite(pinSTBY, LOW);
}
