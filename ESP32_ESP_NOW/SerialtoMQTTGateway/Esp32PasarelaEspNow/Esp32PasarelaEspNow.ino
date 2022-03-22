/*
 * Este código comunica el ESP32 con otros ESP conectados por MAC y usando ESP-NOW como protocolo de comunicación.
 * MASTER MAC: {0x08, 0x3A, 0xF2, 0x6F, 0x13, 0x81}
 * Hacer referencia a MASTER_MAC en códigos para aplicar, así como PMK_KEY_STR / LMK_KEY_STR para intercambio seguro.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
//#include <SoftwareSerial.h>
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
//SoftwareSerial swSer(18, 19, false);
HardwareSerial hwSer(1);

// PMK and LMK keys
uint8_t PMK_KEY_STR[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t LMK_KEY_STR[16] = {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};

// Add below MAC from peers to connect with
uint8_t mac_peer1[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x34};
uint8_t mac_peer2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;

//char para construir el mensaje que se va a enviar por ESP-NOW
char JSON_serie[256];

//Variable para controlar el tiempo.
volatile unsigned long lastMessage = 0;
//Variable para controlar el aviso de "Esperando mensajes ESP-NOW"
unsigned long heartBeat = 0;

hw_timer_t * watchDogTimer = NULL; //watchdog timer declaration
SimpleTimer timerManager; //timer declaration

void IRAM_ATTR watchDogInterrupt();
void watchDogRefresh();
/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  //Encendemos el LED
  digitalWrite(LED_STATUS, HIGH);

  //Preparamos el mensaje para enviarlo por serial. Añadimos la dirección MAC del dispositivo que envía la información.
  //Si el tamaño del mensaje es mayor que 30, es un mensaje con información del sensor, no enviamos con el inicio de $$, si es el mensaje de que está vivo, lo enviamos con %% para diferenciarlos.
  if (data_len > 31) {
    Serial.write("$$");
  }
  else {
    Serial.write("%%");
  }

  Serial.write(mac_addr, 6);
  Serial.write(data_len);
  Serial.write(data, data_len);

  lastMessage = millis();
}

//Método que se encarga de recoger lo que llega del protocolo serie y se envía por ESP-NOW al ESP.
inline void readSerial() {

  //Obtenemos la longitud del JSON que vamos a recibir.
  while (hwSer.available() < 2) {
    delay(1);
  }
  int len1 = (hwSer.read() - 48) * 10;
  int len2 = (hwSer.read() - 48);
  int len = len1 + len2 + 1;

  //Obtenemos el JSON
  while (hwSer.available() < len) {
    delay(1);
  }
  hwSer.readBytes((char*)&JSON_serie, len);
  JSON_serie[len] = '\0'; //fin de cadena, por si acaso

  //Enviamos el JSON por ESP_NOW
  esp_now_send(mac_peer1, (uint8_t *) JSON_serie, strlen(JSON_serie) + 1);
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
  WiFi.mode(WIFI_AP);
  Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());
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

/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  connectToNetworkAndUpdate();

  //init watchdog
  watchDogTimer = timerBegin(0, 80, true); //timer 0, div80
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);

  timerManager.setInterval(CHECK_UPDATE_TIMER * 60000L, connectToNetworkAndUpdate); // Look for update each 10'

  //Inicializamos el LED.
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  //init hwSerial
  hwSer.begin(SERIAL_BAUD_RATE, SERIAL_8N1,17,16);
  
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
}

void loop() {
  timerManager.run();
  checkSerialMessages();
  watchDogRefresh();
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

void checkSerialMessages() {
  //Obtenemos el momento actual.
  unsigned long rightNow = millis();

  //Si la diferencia desde el último heartBeat hasta rightNow es de más de 30 segundos, mostramos un mensaje para hacer referencia que estamos esperando un mensaje de ESP-NOW.
  if (rightNow - heartBeat > 30000) {
    Serial.println("\nEsperando mensajes ESP-NOW...");
    heartBeat = rightNow;
  }

  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LED_STATUS) == HIGH && rightNow - lastMessage >= 200) {
    digitalWrite(LED_STATUS, LOW);
  }

  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (hwSer.available()) {
    //Comprobamos si viene primero el icono '$'
    if ( hwSer.read() == '$' ) {
      //En caso de venir ese icono, comprobamos que el segundo sera también '$'. En caso de ser, es un JSON con información del sensor y vamos a proceder a leerlo.
      while ( ! hwSer.available() )  delay(1);

      if ( hwSer.read() == '$' ) {
        //Leemos los datos.
        readSerial();
        lastMessage = rightNow;
      }
    }
  }
}
/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/
