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
BearSSL::WiFiClientSecure espClient;
PubSubClient client(espClient);
SimpleTimer timerManager; //timer declaration

String topic_string_connect = ("orchard/" + TYPE_NODE + "/");
String topic_string_sub = ("orchard/" + TYPE_NODE + "/");
String topic_string_status = ("orchard/" + TYPE_NODE + "/");

const int systemRelayOutput[] = {4, 5};//4:=relay1;5:=relay2;12:=led1;14:=led2
const int systemLedOutput[] = {12, 14};
int length = sizeof(systemRelayOutput) / sizeof(systemRelayOutput[0]);

int ledState = LOW;
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
unsigned long previousMillis = 0;        // will store last time LED was updated

struct __attribute__((packed)) MQTT_MSG {
  String ctrlMode;
  int numRelay;
  int cmdRelay;
  String id;
} messageReceived;

void irrigationControllerManual(struct MQTT_MSG msgRcv);
void irrigationControllerAuto(struct MQTT_MSG msgRcv);
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup() {

  Serial.begin(115200);
  connectToNetwork();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);

  checkForUpdates();
  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, checkForUpdates); // Look for update each 10'

  ESP.wdtEnable(WATCHDOG_TIMEOUT_S * 1000);

  //Inicializamos el LED y el ID de la placa.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize Rele pinout & LED. Turn off relay
  for (int i = 0; i < length; i++)
  {
    pinMode(systemRelayOutput[i], OUTPUT);
    pinMode(systemLedOutput[i], OUTPUT);
    digitalWrite(systemRelayOutput[i], HIGH);
    digitalWrite(systemLedOutput[i], LOW);
  }
}

void loop() {

  //Obtenemos el momento actual.
  unsigned long rightNow = millis();

  //Si no está conectado al MQTT, nos conectamos.
  if (!client.connected()) reconnect();
  //Esta llamada para que la librería recupere el controlz
  client.loop();

  /*
     Advice system led control:
     If the system is in manual MODE and it's working, advice led has two OP mode:
     1) Blink with 500ms if there're only one relay opened
     2) permanently HIGH if both relay are ON.
  */
  if (messageReceived.ctrlMode.equals("false") && messageReceived.cmdRelay == 1 && messageReceived.numRelay < 4) blinkLed(systemLedOutput[1], rightNow, 500);
  if (messageReceived.ctrlMode.equals("true")) irrigationControllerAuto(rightNow);

  ESP.wdtFeed();
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
  espClient.setFingerprint(OTA_FINGERPRINT);
  espClient.setInsecure();
  espClient.setTimeout(12);
}

void reconnect() {
  String clientId = "ESP8266Client-";
  uint32_t chipID = ESP.getChipId();
  clientId += String(chipID);

  topic_string_connect += (clientId + "/connection");
  const char* conexion_topic = topic_string_connect.c_str();

  topic_string_sub += (clientId + "/activate");
  const char* subTopic = topic_string_sub.c_str();

  topic_string_status += (clientId + "/status");

  // BUCLE DE CHECKING DE CONEXIÓN AL SERVICIO MQTT
  while (!client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");

    // Generación del nombre del cliente en función de la dirección MAC y los ultimos 8 bits del contador temporal

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
  DynamicJsonDocument doc(1024);
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
    String msgString = String(message_buff);
    deserializeJson(doc, msgString);
    JsonObject obj = doc.as<JsonObject>();
    String rele = obj["rele"];
    messageReceived.numRelay = rele.toInt();
    String command = obj["comando"];
    messageReceived.cmdRelay = command.toInt();
    String messageId = obj["id"];
    messageReceived.id = messageId;
    String ctrlMode = obj["auto"];
    messageReceived.ctrlMode = ctrlMode;
    if (messageReceived.ctrlMode.equals("false")) irrigationControllerManual(messageReceived);
  }
}

void irrigationControllerManual(struct MQTT_MSG msgRcv) {
  digitalWrite(systemLedOutput[0], LOW);//Manual mode: static led1 OFF
  switch (msgRcv.cmdRelay) {
    case 0:
      digitalWrite(systemLedOutput[1], LOW);//Manual mode: static led2 OFF
      for (int i = 0; i < length; i++) {
        if (msgRcv.numRelay % 2 == 1) {
          digitalWrite(systemRelayOutput[i], HIGH);//Turn off Relay
        }
        msgRcv.numRelay = msgRcv.numRelay / 2;
      }
      break;
    case 1:
      for (int i = 0; i < length; i++) {
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
  char message[512];
  sprintf(message, "{\"modo\":%i,\"rele\":\"%i\",\"comando\":%i,\"id\":%s}", msgRcv.ctrlMode, msgRcv.numRelay , msgRcv.cmdRelay , msgRcv.id.c_str());
  client.publish(topic_string_status.c_str(), message);
}

void irrigationControllerAuto(unsigned long rightNow) {
  fadingLed(systemLedOutput[0], rightNow, 30); //Auto mode: Fading led1
  switch (messageReceived.cmdRelay) {
    case 0:
      digitalWrite(systemLedOutput[1], LOW);//Manual mode: static led2 OFF
      for (int i = 0; i < length; i++) {
        if (messageReceived.numRelay % 2 == 1) {
          digitalWrite(systemRelayOutput[i], HIGH);//Turn off Relay
        }
        messageReceived.numRelay = messageReceived.numRelay / 2;
      }
      break;
    case 1:
      for (int i = 0; i < length; i++) {
        if (messageReceived.numRelay % 2 == 1) {
          digitalWrite(systemRelayOutput[i], LOW);//Turn on Relay
        }
        else {
          digitalWrite(systemRelayOutput[i], HIGH);//Turn off Relay
        }
        messageReceived.numRelay = messageReceived.numRelay / 2;
      }
      if (messageReceived.numRelay < 4) blinkLed(systemLedOutput[1], rightNow, 500);
      break;
    default:
      break;
  }
  char message[512];
  sprintf(message, "{\"modo\":%i,\"rele\":\"%i\",\"comando\":%i,\"id\":%s}", messageReceived.ctrlMode, messageReceived.numRelay , messageReceived.cmdRelay , messageReceived.id.c_str());
  client.publish(topic_string_status.c_str(), message);
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
    t_httpUpdate_return ret = ESPhttpUpdate.update(espClient, OTA_URL);
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

void blinkLed(int ledPin, unsigned long currentMillis, long interval) {
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

void fadingLed(int ledPin, unsigned long currentMillis, long interval) {
  if (currentMillis - previousMillis >= interval) {
    // set the brightness of pin 9:
    analogWrite(ledPin, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }
    previousMillis = currentMillis;
  }
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
