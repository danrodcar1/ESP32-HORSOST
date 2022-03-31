/*

  Envía un JSON a la pasarella ESP-NOW -> MQTT que está escuchando en la MAC 3E:33:33:33:33:33
  Después del envío o si hay un error o time-out el dispositivo entra en deep-sleep durante 30 segundos + o -

  IMPORTANTE !!!
  Para que el dispositivo se despierte desde deep-sleep hay que conectar el GPIO16 con RST (reset)
  Pero para programar la placa hay que quitar esa conexión con RST

*/

//Librerias
#include <WiFi.h>
#include <esp_now.h>
#include "DHTesp.h"
#include <esp_wifi.h>
DHTesp dht;

// GPIOs
#define LEDCheck 2
#define LEDSalida 15
#define DHT 5

//canal que se va a utilizar para el ESP-NOW
#define WIFI_CHANNEL 0
//2 segundos que va a estar enviando la placa información de que está viva
#define ALIVE_SECS 2  // segundos
//2 segundos máximo para dar TIME_OUT e irse a dormir
#define SEND_TIMEOUT 2000

//Claves que necesitamos para encriptar la comunicación por ESP-NOW
uint8_t PMK_KEY_STR[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t LMK_KEY_STR[16] = {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};

//MAC de la ESP remota que recibe los mensajes de esta ESP mediante ESP-NOW. 08:3A:F2:A8:11:75

uint8_t macPasarela[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
//MAC de la ESP que está en la pasarela y se encarga del ESP-NOW
uint8_t macDHT11[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x34};
//Variable dónde se especifica el tiempo que se deja entre actualización y actualización del DHT11.
int frecuenciaActualizacion = 30;
//char para construir el mensaje que se va a enviar por ESP-NOW
char mensaje[512];
//Variables para controlar el tiempo.
unsigned long waitMs = 0;
//Variable que se utilizará para controlar en tiempo cada envío de información.
int countTime = 5000;
int countAliveTime = 0;
//Valor actual del LED de salida
int currentLEDValue = 0;
//Valor que llega por ESP-NOW del LED de salida
int espNowLEDValue = 0;

esp_now_peer_info_t peer1;


void setup() {

  Serial.begin(115200);
  Serial.println();

  //Inicializamos el LED y el DHT11.
  pinMode(LEDCheck, OUTPUT);
  pinMode(LEDSalida, OUTPUT);
  dht.setup(DHT, DHTesp::DHT11);

  //Ponemos el WiFi en modo AP.
  WiFi.mode(WIFI_AP);
  //  WiFi.disconnect();
  //Establecemos la MAC para esta ESP
  esp_wifi_set_mac(ESP_IF_WIFI_AP, macDHT11);
  Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());

  //Inicializamos ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("*** Fallo al iniciar el ESP-NOW");
    ESP.restart();
  }

  //Especificamos las claves para enviar la información de manera encriptada.
  esp_now_set_pmk(PMK_KEY_STR);

  memcpy(peer1.peer_addr, macPasarela, 6);
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
  //Comprueba si un paquete de datos enviado ha sido recibido correctamente por un par
  esp_now_register_send_cb(OnDataSent);

  //Si llega un mensaje por ESP-NOW a esta ESP, cambiamos el valor del LED.
  esp_now_register_recv_cb(OnRecv);
}

void loop() {

  //Un condicional para controlar cada cuanto tiempo se envía información del sensor.
  if (countTime < millis()) {

    //Encedemos el LED
    digitalWrite(LEDCheck, HIGH);

    //Esperamos a que el sensor esté preparado para poder leerlo.
    delay(dht.getMinimumSamplingPeriod());

    //Leemos el sensor.
    float t = dht.getTemperature();
    t = (isnan(t)) ? -255 : t;
    float h = dht.getHumidity();
    h = (isnan(h)) ? -255 : h;

    //Creamos y mostramos el JSON que se va a enviar.
    sprintf(mensaje, "orchard/espnow/datos|{\"hum\":%g,\"temp\":%g,\"status\":\"%s\"}", h, t, dht.getStatusString());
    Serial.printf("Mensaje: %s\n", mensaje);

    //Enviamos el JSON por ESP_NOW
    esp_now_send(macPasarela, (uint8_t *) mensaje, strlen(mensaje) + 1);

    //Apagamos el LED
    digitalWrite(LEDCheck, LOW);

    //Calculamos el tiempo para envíar la siguiente información.
    countTime = millis() + frecuenciaActualizacion * 1000;

    //Si da la casualidad de que, después de enviar los datos del sensor, también va a enviar el mensaje de que está vivo, lo retrasamos un segundo para que no haya interferencias.
    if (countAliveTime != 0 && countAliveTime < millis()) {
      countAliveTime = millis() + 1000;
    }
  }

  //Un condicional para controlar cada cuanto tiempo se envía un keepalive.
  if (countAliveTime < millis()) {

    //Encedemos el LED
    digitalWrite(LEDCheck, HIGH);

    //Creamos y mostramos el JSON que se va a enviar.
    sprintf(mensaje, "orchard/espnow/estado|{\"status\":\"alive\",\"freq\":\"%i\"}", frecuenciaActualizacion);
    Serial.printf("Mensaje: %s\n", mensaje);

    //Enviamos el JSON por ESP_NOW
    esp_now_send(macPasarela, (uint8_t *) mensaje, strlen(mensaje) + 1);

    //Apagamos el LED
    digitalWrite(LEDCheck, LOW);

    //Calculamos el tiempo para envíar la siguiente información.
    countAliveTime = millis() + ALIVE_SECS * 1000;
  }

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


// callback function executed when data is received
void OnRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  Serial.printf("\r\nReceived\t%d Bytes\t%d", data_len, data[0]);
}
