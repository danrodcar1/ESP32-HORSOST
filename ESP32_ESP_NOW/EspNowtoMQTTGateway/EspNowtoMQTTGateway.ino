#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "myDefines.h"
#include <ESP8266httpUpdate.h>

extern "C" {
#include "user_interface.h"
#include <espnow.h>
}

#define SET_SECURE_WIFI false

#if SET_SECURE_WIFI == false
#include <WiFiClient.h>
WiFiClient espClient;
#define MQTT_SERVER         "10.10.10.10"
#define MQTT_PORT           1883
#define MQTT_USER           NULL
#define MQTT_PASSWORD       NULL
#else
#include <WiFiClientSecure.h>
BearSSL::WiFiClientSecure espClient;
#define MQTT_SERVER         "huertociencias.uma.es"
#define MQTT_PORT           8163
#define MQTT_USER           "huerta"
#define MQTT_PASSWORD       "accesohuertica"
#endif
/**********************************************************************
   VARS
***********************************************************************/

PubSubClient client (espClient);

// PMK and LMK keys
uint8_t PMK_KEY_STR[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t LMK_KEY_STR[16] = {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};


uint8_t gatewayCustomMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33}; //Custom mac address for
uint8_t macDHT11[] = {0xDC, 0x4F, 0x22, 0x76, 0x23, 0x47};// your MAC

struct __attribute__((packed)) PEER_INFO {
  String macaddr;
  String message;
  String topic;
} messageReceived;

volatile boolean haveReading = false;
int heartBeat;

/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup() {
  Serial.begin(115200);
  client.setServer(MQTT_SERVER, MQTT_PORT);

  Serial.println();
  Serial.println();
  Serial.println("ESP_Now Controller");
  Serial.println();

  WiFi.mode(WIFI_AP);

  wifi_set_macaddr(SOFTAP_IF, &gatewayCustomMac[0]);

  Serial.print("This node AP mac: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: "); Serial.println(WiFi.macAddress());

  initEspNow();
  Serial.println("Setup done");
}


void loop() {
  if (millis() - heartBeat > 30000) {
    Serial.println("Waiting for ESP-NOW messages...");
    heartBeat = millis();
  }

  if (haveReading) {
    haveReading = false;
    wifiConnect();
    reconnectMQTT();
    sendToBroker();
    client.disconnect();
    delay(200);
    ESP.restart(); // <----- Reboots to re-enable ESP-NOW
  }
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/

void initEspNow() {
  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }
  esp_now_set_kok(PMK_KEY_STR, 16);
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(macDHT11, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, LMK_KEY_STR, 0);
  //Especificamos las claves para enviar la información de manera encriptada.
  esp_now_set_peer_key(gatewayCustomMac, LMK_KEY_STR, 16);

  esp_now_register_recv_cb(OnDataRecv);
}

void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {
  Serial.println();
  Serial.println("=== Data Received from ===");
  Serial.print("Mac address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(mac_addr[i], HEX);
    if (i < 5)    Serial.print(":");
  }
  Serial.println();

  for (int i = 0; i < 6; i++) messageReceived.macaddr += byte2HEX(mac_addr[i]);
  for (auto & c : messageReceived.macaddr) c = toupper(c);

  //Obtenemos el mensaje formado como topic|mensaje
  char message[data_len];
  for (int i = 0; i < data_len; i++) {
    message[i] = data[i];
  }
  message[data_len] = '\0'; //fin de cadena, por si acaso
  String serialmsg = String(message);
  messageReceived.topic = serialmsg.substring(0, serialmsg.indexOf("|")) + '\0';
  messageReceived.message = serialmsg.substring(serialmsg.indexOf("|") + 1, data_len) + '\0';
  Serial.print("Topic to send: ");
  Serial.println(messageReceived.topic);
  Serial.print("Message to send: ");
  Serial.println(messageReceived.message);
  haveReading = true;
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected, IP address: "); Serial.println(WiFi.localIP());
//  espClient.setFingerprint(OTA_FINGERPRINT);
//  espClient.setInsecure();
//  espClient.setTimeout(12);
  checkForUpdates();
}

void sendToBroker() {

  //Build mqtt topic from incoming message and public
  String pub_topic = ("orchard/" + TYPE_NODE + "/" + messageReceived.macaddr + "/" + messageReceived.topic);    // Select topic by ESP MAC
  const char* send_topic = pub_topic.c_str();
  char mensaje_mqtt[512];
  sprintf(mensaje_mqtt, "{\"mac\":\"%s\",\"mensaje\":%s}", messageReceived.macaddr.c_str(), messageReceived.message.c_str());
  Serial.println("Publish");
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.publish(send_topic, mensaje_mqtt);
}


void reconnectMQTT() {
  String clientId = "ESP8266Client-";
  uint32_t chipID = ESP.getChipId();
  clientId += String(chipID);
  String topic_string_connect = ("orchard/" + TYPE_NODE + "/" + clientId + "/connection");      // Select topic by ESP ID : connection
  const char* conexion_topic = topic_string_connect.c_str();
  Serial.println(" Loop until we're reconnected");
  while (!client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");
    // Attempt to connect
    Serial.print("Conectando a ");
    Serial.print(MQTT_SERVER);
    Serial.print(" como ");
    Serial.println(clientId);
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD, conexion_topic, 2, true, "Offline", true)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(conexion_topic, "Online", true);
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
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
