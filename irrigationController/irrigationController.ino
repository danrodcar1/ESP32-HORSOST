//Librerias.
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include "myDefines.h"
#include "build_defs.h"
#include <SimpleTimer.h> //repetitive tasks
#include <EEPROM.h>
#include <WiFiClientSecure.h>

/**********************************************************************
   VARS
***********************************************************************/
WiFiClientSecure espClient;
PubSubClient client(espClient);
SimpleTimer timerManager; //timer declaration

String clientId = "ESP8266Client-";
String topic_string_connect = ("orchard/" + TYPE_NODE + "/");
String topic_string_sub = ("orchard/" + TYPE_NODE + "/");
String topic_string_status = ("orchard/" + TYPE_NODE + "/");
String topic_string_time = ("orchard/" + TYPE_NODE + "/");

const int systemRelayOutput[] = {4, 5};//4:=relay1;5:=relay2;12:=led1;14:=led2
const int systemLedOutput[] = {12, 14};
int lengthOutput = sizeof(systemRelayOutput) / sizeof(systemRelayOutput[0]);

int ledState = LOW;
int blinkLedTimer = 0;

struct __attribute__((packed)) MQTT_MSG {
  bool ctrlMode;
  int numRelay;
  int cmdRelay;
  const char* id;
} messageReceived;

unsigned long lwdpreviousMillis;

void irrigationControllerManual(struct MQTT_MSG msgRcv);
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup() {

  Serial.begin(115200);
  connectToNetwork();
  createTopic();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);

  checkForUpdates();
  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, checkForUpdates); // Look for update each 10'

  blinkLedTimer = timerManager.setInterval(500, blinkLed);
  timerManager.disable(blinkLedTimer);

  //Inicializamos el LED y el ID de la placa.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize Rele pinout & LED. Turn off relay
  for (int i = 0; i < lengthOutput; i++)
  {
    pinMode(systemRelayOutput[i], OUTPUT);
    pinMode(systemLedOutput[i], OUTPUT);
    digitalWrite(systemRelayOutput[i], HIGH);
    digitalWrite(systemLedOutput[i], LOW);
  }
  lwdpreviousMillis = millis();
}

void loop() {

  //Obtenemos el momento actual.
  unsigned long rightNow = millis();

  //Si no está conectado al MQTT, nos conectamos.
  if (!client.connected()) reconnect();
  //Esta llamada para que la librería recupere el controlz
  client.loop();

  if ((unsigned long)(rightNow - lwdpreviousMillis) >= 30L * 60000L)
  {
    char time_Buff[100];
    sprintf(time_Buff, "%lu", rightNow);
    client.publish(topic_string_time.c_str(), time_Buff);
    lwdpreviousMillis = millis();
  }

  timerManager.run();
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/


void connectToNetwork() {
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

void createTopic() {  
  uint32_t chipID = ESP.getChipId();
  clientId += String(chipID);
  topic_string_connect += (clientId + "/connection");
  topic_string_sub += (clientId + "/activate");
  topic_string_time += (clientId + "/progTime");
  topic_string_status += (clientId + "/status");
}

void reconnect() {
  // BUCLE DE CHECKING DE CONEXIÓN AL SERVICIO MQTT
  while (!client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");

    // Generación del nombre del cliente en función de la dirección MAC y los ultimos 8 bits del contador temporal

    Serial.print("Conectando a ");
    Serial.print(MQTT_SERVER);
    Serial.print(" como ");
    Serial.println(clientId);

    // Intentando conectar
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, topic_string_connect.c_str(), 2, true, "Offline", true)) {
      Serial.println("conectado");
      // Nos suscribimos a los siguientes topics
      client.subscribe(topic_string_sub.c_str());
      Serial.printf("\r\Subscribed to:\t%s", topic_string_sub.c_str());
      Serial.println();
      // Publicamos el estado de la conexion en el topic
      client.publish(topic_string_connect.c_str(), "Online", true);
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
    messageReceived.ctrlMode =  doc["auto"];
    messageReceived.id = doc["id"];
    messageReceived.numRelay = doc["rele"];
    messageReceived.cmdRelay  = doc["comando"];
    client.publish(topic_string_status.c_str(), message_buff);
    irrigationController(messageReceived);
  }
}

void irrigationController(struct MQTT_MSG msgRcv) {
  if (msgRcv.ctrlMode == true) digitalWrite(systemLedOutput[0], HIGH); //Auto mode: static led1 ON
  else digitalWrite(systemLedOutput[0], LOW);//Manual mode: static led1 OFF
  switch (msgRcv.cmdRelay) {
    case 0:
      timerManager.disable(blinkLedTimer);
      digitalWrite(systemLedOutput[1], LOW);
      for (int i = 0; i < lengthOutput; i++) {
        if (msgRcv.numRelay % 2 == 1) {
          digitalWrite(systemRelayOutput[i], HIGH);//Turn off Relay
        }
        msgRcv.numRelay = msgRcv.numRelay / 2;
      }
      break;
    case 1:
      timerManager.enable(blinkLedTimer);
      for (int i = 0; i < lengthOutput; i++) {
        if (msgRcv.numRelay % 2 == 1) {
          digitalWrite(systemRelayOutput[i], LOW);//Turn on Relay
        }
        else {
          digitalWrite(systemRelayOutput[i], HIGH);//Turn off Relay
        }
        msgRcv.numRelay = msgRcv.numRelay / 2;
      }
      break;
    default:
      break;
  }
}

void checkForUpdates() {
  // wait for WiFi connection
  if ((WiFi.status() == WL_CONNECTED)) {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);
    t_httpUpdate_return ret = ESPhttpUpdate.update(espClient, OTA_URL, HTTP_OTA_VERSION);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}

void blinkLed() {
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  // set the LED with the ledState of the variable:
  digitalWrite(systemLedOutput[1], ledState);
}

/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/

void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/
