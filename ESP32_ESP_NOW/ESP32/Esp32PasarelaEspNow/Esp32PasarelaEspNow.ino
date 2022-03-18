// DISPOSITIVO MAESTRO

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SoftwareSerial.h>
#include "myDefines.h"
#include "build_defs.h"



/**********************************************************************
   VARS
***********************************************************************/
#define BAUD_RATE 115200
#define LEDCheck 2

//canal que se va a utilizar para el ESP-NOW
#define WIFI_CHANNEL 6

//Configuramos el protocolo serie.
SoftwareSerial swSer(14, 12, false);

//master MAC addr
uint8_t masterCustomMac[] = {0xB4, 0xE6, 0x2D, 0xB2, 0x1B, 0x36};

// Add below MAC from peers to connect with
uint8_t mac_peer1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t mac_peer2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;

//char para construir el mensaje que se va a enviar por ESP-NOW
char JSON_serie[256];

//Variable para controlar el tiempo.
volatile unsigned long ultimo_mensaje = 0;
//Variable para controlar el aviso de "Esperando mensajes ESP-NOW"
unsigned long heartBeat = 0;

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  //Encendemos el LED
  digitalWrite(LEDCheck, HIGH);

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

  ultimo_mensaje = millis();
}

//Método que se encarga de recoger lo que llega del protocolo serie y se envía por ESP-NOW al ESP.
inline void readSerial() {

  //Obtenemos la longitud del JSON que vamos a recibir.
  while (swSer.available() < 2) {
    delay(1);
  }
  int len1 = (swSer.read() - 48) * 10;
  int len2 = (swSer.read() - 48);
  int len = len1 + len2 + 1;

  //Obtenemos el JSON
  while (swSer.available() < len) {
    delay(1);
  }
  swSer.readBytes((char*)&JSON_serie, len);
  JSON_serie[len] = '\0'; //fin de cadena, por si acaso

  //Enviamos el JSON por ESP_NOW
  //  esp_now_send(macDHT11, (uint8_t *) JSON_serie, strlen(JSON_serie) + 1);
}

void addingMoreFriends() {
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = WIFI_CHANNEL;
  peer1.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 1");
  if ( esp_now_add_peer(&peer1) == ESP_OK) {
    Serial.println("Peer 1 added");
  }
  
  memcpy(peer2.peer_addr, mac_peer2, 6);
  peer2.channel = WIFI_CHANNEL;
  peer2.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 2");
  if ( esp_now_add_peer(&peer2) == ESP_OK) {
    Serial.println("Peer 2 added");
  }
}
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  Serial.println();

  //Inicializamos el LED.
  pinMode(LEDCheck, OUTPUT);
  digitalWrite(LEDCheck, LOW);

  //Ponemos el WiFi en modo STA (Master).
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //Establecemos la MAC para esta ESP
  esp_wifi_set_mac(ESP_IF_WIFI_STA, &masterCustomMac[0]);
  Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP NOW INIT FAILED.... REBOOT");
    ESP.restart();
  }
  addingMoreFriends();
  //Si llega un mensaje por ESP-NOW a esta ESP, recogemos la información y la enviamos por serial.
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  //Obtenemos el momento actual.
  unsigned long ahora = millis();

  //Si la diferencia desde el último heartBeat hasta ahora es de más de 30 segundos, mostramos un mensaje para hacer referencia que estamos esperando un mensaje de ESP-NOW.
  if (ahora - heartBeat > 30000) {
    Serial.println("\nEsperando mensajes ESP-NOW...");
    heartBeat = ahora;
  }

  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LEDCheck) == HIGH && ahora - ultimo_mensaje >= 200) {
    digitalWrite(LEDCheck, LOW);
  }

  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (swSer.available()) {
    //Comprobamos si viene primero el icono '$'
    if ( swSer.read() == '$' ) {
      //En caso de venir ese icono, comprobamos que el segundo sera también '$'. En caso de ser, es un JSON con información del sensor y vamos a proceder a leerlo.
      while ( ! swSer.available() )  delay(1);

      if ( swSer.read() == '$' ) {
        //Leemos los datos.
        readSerial();
        ultimo_mensaje = ahora;
      }
    }
  }
}





/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/



/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/
