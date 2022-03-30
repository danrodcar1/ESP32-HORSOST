

/**
   Recibe por serie un mensaje que también contiene la MAC del emisor original y lo publica en MQTT
   Publica en el topic infind/espnow/MAC en un servidor local 192.168.1.101
*/

//Librerias.
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include "myDefines.h"
#include "build_defs.h"
#include <SimpleTimer.h> //repetitive tasks
#include <EEPROM.h>

//#include <WiFiClientSecure.h>
//BearSSL::WiFiClientSecure espClient;
//#define MQTT_SERVER         "huertociencias.uma.es"
//#define MQTT_PORT           8163
//#define MQTT_USER           "huerta"
//#define MQTT_PASSWORD       "accesohuertica"

#include <WiFiClient.h>
WiFiClient espClient;
#define MQTT_SERVER         "10.10.10.10"
#define MQTT_PORT           1883
#define MQTT_USER           NULL
#define MQTT_PASSWORD       NULL

/**********************************************************************
   VARS
***********************************************************************/

PubSubClient client(espClient);
SimpleTimer timerManager; //timer declaration

//Variable para controlar el aviso de "Esperando mensajes ESP-NOW".
unsigned long heartBeat = 0;
//Variable para controlar el tiempo.
unsigned long lastMessage = 0;

struct serialReceived {
  String macaddr;
  byte len;
  String message;
  String topic;
};

/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup() {

  Serial.begin(115200);
  connectToNetwork();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(512);

  //  checkForUpdates();
  //  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, checkForUpdates); // Look for update each 10'

  ESP.wdtEnable(WATCHDOG_TIMEOUT_S * 1000);

  //Inicializamos el LED y el ID de la placa.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  //Obtenemos el momento actual.
  unsigned long rightNow = millis();

  //Si no está conectado al MQTT, nos conectamos.
  if (!client.connected()) reconnect();
  //Esta llamada para que la librería recupere el controlz
  client.loop();

  readSerialAndPublicMQTT(rightNow);
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
  //  espClient.setFingerprint(OTA_FINGERPRINT);
  //  espClient.setInsecure();
  //  espClient.setTimeout(12);

}

void reconnect() {
  String clientId = "ESP8266Client-";
  uint32_t chipID = ESP.getChipId();
  clientId += String(chipID);
  String topic_string_connect = ("orchard/" + TYPE_NODE + "/" + clientId + "/connection");      // Select topic by ESP ID : connection
  const char* conexion_topic = topic_string_connect.c_str();

  String topic_string_sub = ("orchard/" + TYPE_NODE);      // Select topic by ESP ID : connection
  const char* publicTopic = topic_string_sub.c_str();

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
      client.subscribe(publicTopic);
      Serial.printf("\r\Subscribed to:\t%s", publicTopic);
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
  if (strcmp(topic, "orchard/espnow") == 0)
  {
    int i;
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    String msgString = String(message_buff);
    deserializeJson(doc, msgString);
    JsonObject obj = doc.as<JsonObject>();
    String macaddr = obj["MAC"] + '\0';
    String topic = obj["topic"];
    String message = obj["message"];
    String packet = (topic + "|" + message);    // Select topic by ESP MAC
    Serial.write("$$");
    Serial.write(HexString2ASCIIString(macaddr).c_str());
    Serial.write(packet.length());
    Serial.write(packet.c_str());
  }
}


void readSerialAndPublicMQTT(unsigned long rightNow) {
  serialReceived readSerial;
  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (Serial.available()) {
    //Comprobamos si vienen mensajes con datos, empiezan con '$$'
    if ( Serial.read() == '$' ) {
      while ( ! Serial.available() )  delay(1);
      if ( Serial.read() == '$' ) {
        //Enciende el led al enviar mensaje
        digitalWrite(LED_BUILTIN, HIGH);
        //Extrac MAC, topic and data from Serial : MAC/{data}
        readSerial = readFromSerial();
        //Build mqtt topic from incoming message and public
        String pub_topic = ("orchard/" + TYPE_NODE + "/" + readSerial.macaddr + "/" + readSerial.topic);    // Select topic by ESP MAC
        const char* send_topic = pub_topic.c_str();
        char mensaje_mqtt[512];
        sprintf(mensaje_mqtt, "{\"mac\":\"%s\",\"mensaje\":%s}", readSerial.macaddr.c_str(), readSerial.message.c_str());
        client.publish(send_topic, mensaje_mqtt);
        lastMessage = rightNow;
      }
    }
  }
  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LED_BUILTIN) == HIGH && rightNow - lastMessage >= 200) {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

serialReceived readFromSerial() {
  serialReceived readSerial;
  //Obtenemos la dirección MAC.
  while (Serial.available() < 6) {
    delay(1);
  }
  for (int i = 0; i < 6; i++) readSerial.macaddr += byte2HEX(Serial.read());
  for (auto & c : readSerial.macaddr) c = toupper(c);

  //Obtenemos la longitud del mensaje que vamos a recibir.
  while (Serial.available() < 1) {
    delay(1);
  }
  readSerial.len = Serial.read();

  //Obtenemos el mensaje formado como topic|mensaje
  char message[readSerial.len];
  while (Serial.available() < readSerial.len) {
    delay(1);
  }
  Serial.readBytes(message, readSerial.len);
  message[readSerial.len] = '\0'; //fin de cadena, por si acaso
  String serialmsg = String(message);
  readSerial.topic = serialmsg.substring(0, serialmsg.indexOf("|")) + '\0';
  readSerial.message = serialmsg.substring(serialmsg.indexOf("|") + 1, readSerial.len) + '\0';
  return readSerial;
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
   UTILITY FUNCTIONS
***********************************************************************/

//Hex to ASCII converter
String HexString2ASCIIString(String hexstring) {
  String temp = "", sub = "", result;
  char buf[3];
  for (int i = 0; i < hexstring.length(); i += 2) {
    sub = hexstring.substring(i, i + 2);
    sub.toCharArray(buf, 3);
    char b = (char)strtol(buf, 0, 16);
    if (b == '\0')
      break;
    temp += b;
  }
  return temp;
}

//Byte to hex converter
inline String byte2HEX (byte data)
{
  return (String(data, HEX).length() == 1) ? String("0") + String(data, HEX) : String(data, HEX);
}




/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/
