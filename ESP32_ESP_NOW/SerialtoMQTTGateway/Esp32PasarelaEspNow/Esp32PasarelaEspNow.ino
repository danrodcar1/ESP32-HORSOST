/*
   Este código comunica el ESP32 con otros ESP conectados por MAC y usando ESP-NOW como protocolo de comunicación.
   MASTER MAC: {0x08, 0x3A, 0xF2, 0x6F, 0x13, 0x81}
   Hacer referencia a MASTER_MAC en códigos para aplicar, así como PMK_KEY_STR / LMK_KEY_STR para intercambio seguro.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "myDefines.h"
#include "build_defs.h"
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include <SimpleTimer.h> //repetitive tasks
#include "esp_system.h" //watchdog


/**********************************************************************
   VARS
***********************************************************************/

WiFiClientSecure espClient;

// PMK and LMK keys
uint8_t PMK_KEY_STR[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t LMK_KEY_STR[16] = {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};

uint8_t gatewayCustomMac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33}; //Custom mac address for

// Add below MAC from peers to connect with
uint8_t mac_peer1[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x34};
uint8_t mac_peer2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;

struct serialReceived {
  String macaddr;
  byte len;
  String message;
  String topic;
  String packet;
};

//Variable para controlar el tiempo.
volatile unsigned long lastMessage = 0;
//Variable para controlar el aviso de "Esperando mensajes ESP-NOW"
unsigned long heartBeat = 0;

hw_timer_t * watchDogTimer = NULL; //watchdog timer declaration
SimpleTimer timerManager; //timer declaration

void IRAM_ATTR watchDogInterrupt();
void watchDogRefresh();

/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  connectToNetworkAndUpdate();
  //Ponemos el WiFi en modo AP.
  WiFi.mode(WIFI_AP);
  //Establecemos la MAC para esta ESP
  esp_wifi_set_mac(ESP_IF_WIFI_AP, gatewayCustomMac); // esp32 code
  Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());
  //init watchdog
  watchDogTimer = timerBegin(0, 80, true); //timer 0, div80
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);

  //  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, connectToNetworkAndUpdate); // Look for update each 10'

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP NOW INIT FAILED.... REBOOT");
    ESP.restart();
  }
  // Setting the PMK key
  esp_now_set_pmk(PMK_KEY_STR);
  addingMoreFriends();
  //Si llega un mensaje por ESP-NOW a esta ESP, recogemos la información y la enviamos por serial.
  esp_now_register_recv_cb(OnDataRecv);
  // We will register the callback function to respond to the event
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  unsigned long rightNow = millis();
  timerManager.run();
  readSerialtoEspnow(rightNow);
  watchDogRefresh();
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/

void readSerialtoEspnow(unsigned long rightNow) {
  serialReceived readSerial;
  //Si la diferencia desde el último heartBeat hasta rightNow es de más de 30 segundos, mostramos un mensaje para hacer referencia que estamos esperando un mensaje de ESP-NOW.
  if (rightNow - heartBeat > 30000) {
    Serial.println("\nEsperando mensajes ESP-NOW...");
    heartBeat = rightNow;
  }

  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (Serial.available()) {
    //Comprobamos si viene primero el icono '$'
    if ( Serial.read() == '$' ) {
      while ( ! Serial.available() )  delay(1);
      if ( Serial.read() == '$' ) {
        //Leemos los datos.
        //Extrac MAC, topic and data from Serial : MAC/{data}
        readSerial = readFromSerial();
        //Enviamos por ESP_NOW
        esp_now_send((uint8_t*)HexString2ASCIIString(readSerial.macaddr).c_str(), (uint8_t*)readSerial.packet.c_str(), 250);
        lastMessage = rightNow;
      }
    }
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.write("$$");
  Serial.write(mac_addr, 6);
  Serial.write(data_len);
  Serial.write(data, data_len);
  lastMessage = millis();
}

// Callback to have a track of sent messages
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println();
  Serial.println("=== Data Sent to ===");
  Serial.print("Mac address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(mac_addr[i], HEX);
    if (i < 5)    Serial.print(":");
  }
  Serial.print("\r\nSend message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent Successfully" : "Sent Failed");
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
  readSerial.packet = String(message);
  readSerial.topic = readSerial.packet.substring(0, readSerial.packet.indexOf("|")) + '\0';
  readSerial.message = readSerial.packet.substring(readSerial.packet.indexOf("|") + 1, readSerial.len) + '\0';
  return readSerial;
}




void addingMoreFriends() {
  //adding encrypted friends
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = WIFI_CHANNEL;
  // Setting the master device LMK key
  for (uint8_t i = 0; i < 16; i++) {
    peer1.lmk[i] = LMK_KEY_STR[i];
  }
  peer1.encrypt = true;
  peer1.ifidx = ESP_IF_WIFI_AP; //CLAVE
  // Register the peer
  Serial.println("Registering a peer 1");
  if ( esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("There was an error registering the slave");
    return;
  } else {
    Serial.println("Peer 1 added");
  }

  //adding normal friends
  memcpy(peer2.peer_addr, mac_peer2, 6);
  peer2.channel = WIFI_CHANNEL;
  peer2.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 2");
  if ( esp_now_add_peer(&peer2) != ESP_OK) {
    Serial.println("There was an error registering the slave");
    return;
  } else {
    Serial.println("Peer 2 added");
  }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToNetworkAndUpdate() {
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
  checkForUpdates();
  WiFi.disconnect();
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
