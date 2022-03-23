/**
   Recibe por serie un mensaje que también contiene la MAC del emisor original y lo publica en MQTT
   Publica en el topic infind/espnow/MAC en un servidor local 192.168.1.101
*/

//Librerias.
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include "myDefines.h"
#include "build_defs.h"
#include <SimpleTimer.h> //repetitive tasks

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

//Cadenas para topics e ID.
uint8_t macAddr[6];

char topic_PUB[256];
char mensaje_mqtt[512];
char JSON_serie[256];
String deviceMac;

//Variable para controlar el aviso de "Esperando mensajes ESP-NOW".
unsigned long heartBeat = 0;
//Variable para controlar el tiempo.
unsigned long lastMessage = 0;

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
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
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
  char message_buff[100];
  int i;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);

  Serial.write("$$");
  Serial.println(length);
  Serial.println(msgString);
}



/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/

void readSerialAndPublicMQTT(unsigned long rightNow) {
  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (Serial.available()) {
    //Comprobamos si vienen mensajes con datos, empiezan con '$$'
    if ( Serial.read() == '$' ) {
      while ( ! Serial.available() )  delay(1);
      if ( Serial.read() == '$' ) {
        //Enciende el led al enviar mensaje
        digitalWrite(LED_STATUS, HIGH);
        //Extrac MAC and data from Serial : MAC/{data}
        String serialmsg = Serial.readString();
        String macaddr = Serial.readStringUntil('/');
        String datastr = serialmsg.substring(serialmsg.indexOf("/") + 1, serialmsg.length()) + '\0';

        //Build mqtt topic from incoming message and public
        String mac_topic = ("orchard/" + TYPE_NODE + "/" + macaddr + "/datos");    // Select topic by ESP MAC
        const char* send_topic = mac_topic.c_str();
        sprintf(mensaje_mqtt, "{\"mac\":\"%s\",\"mensaje\":%s}", macaddr, datastr);
        client.publish(send_topic, mensaje_mqtt);

        lastMessage = rightNow;
      }
    }

    //Comprobamos si vienen mensajes de estado, empiezan con '%%'
    if ( Serial.read() == '%' ) {
      while ( ! Serial.available() )  delay(1);
      if ( Serial.read() == '%' ) {
        //Enciende el led al enviar mensaje
        digitalWrite(LED_STATUS, HIGH);
        //Extrac MAC and data from Serial : MAC/{status}
        String serialmsg = Serial.readString();
        String macaddr = Serial.readStringUntil('/');
        String datastr = serialmsg.substring(serialmsg.indexOf("/") + 1, serialmsg.length()) + '\0';

        //Build mqtt topic from incoming message and public
        String mac_topic = ("orchard/" + TYPE_NODE + "/" + macaddr + "/status");    // Select topic by ESP MAC
        const char* send_topic = mac_topic.c_str();
        sprintf(mensaje_mqtt, "{\"mac\":\"%s\",\"mensaje\":%s}", macaddr, datastr);
        client.publish(send_topic, mensaje_mqtt);

        lastMessage = rightNow;
      }
    }
  }
  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LED_STATUS) == HIGH && rightNow - lastMessage >= 200) {
    digitalWrite(LED_STATUS, LOW);
  }
}

//Método que devuelve 2 caracteres HEX para un byte.
inline String byte2HEX (byte data)
{
  return (String(data, HEX).length() == 1) ? String("0") + String(data, HEX) : String(data, HEX);
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
   INTERRUPT FUNCTIONS
***********************************************************************/
