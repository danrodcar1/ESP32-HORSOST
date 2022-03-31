/* 

 Envía un JSON a la pasarella ESP-NOW -> MQTT que está escuchando en la MAC 3E:33:33:33:33:33
 Después del envío o si hay un error o time-out el dispositivo entra en deep-sleep durante 30 segundos + o -

 IMPORTANTE !!!
 Para que el dispositivo se despierte desde deep-sleep hay que conectar el GPIO16 con RST (reset)
 Pero para programar la placa hay que quitar esa conexión con RST
 
 */
 
//Librerias
#include <ESP8266WiFi.h> 
#include <espnow.h>
#include "DHTesp.h"
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
uint8_t kok[16]= {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t key[16]= {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};

//MAC de la ESP remota que recibe los mensajes de esta ESP mediante ESP-NOW. 08:3A:F2:A8:11:75

uint8_t macPasarela[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33};
//MAC de la ESP que está en la pasarela y se encarga del ESP-NOW
uint8_t macDHT11[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x34};
//Variable dónde se especifica el tiempo que se deja entre actualización y actualización del DHT11.
int frecuenciaActualizacion=30;
//char para construir el mensaje que se va a enviar por ESP-NOW
char mensaje[512];
//Variables para controlar el tiempo.
unsigned long waitMs=0;
//Variable que se utilizará para controlar en tiempo cada envío de información.
int countTime = 5000;
int countAliveTime = 0;
//Valor actual del LED de salida
int currentLEDValue = 0;
//Valor que llega por ESP-NOW del LED de salida
int espNowLEDValue = 0;

void setup() {
	
	Serial.begin(115200); 
	Serial.println();
	
	//Inicializamos el LED y el DHT11.
	pinMode(LEDCheck, OUTPUT);  
  pinMode(LEDSalida, OUTPUT); 
	dht.setup(DHT, DHTesp::DHT11);
	
	//Put WiFi in AP mode.
	WiFi.mode(WIFI_AP);
  //Establecemos la MAC para esta ESP
  wifi_set_opmode(STATIONAP_MODE);
	wifi_set_macaddr(SOFTAP_IF, &macDHT11[0]);
	Serial.print("MAC: "); Serial.println(WiFi.softAPmacAddress());

  //Inicializamos ESP-NOW
	if (esp_now_init() != 0) {
		Serial.println("*** Fallo al iniciar el ESP-NOW");
	  ESP.restart();
	}

	//Especificamos las claves para enviar la información de manera encriptada.
  esp_now_set_kok(kok, 16);
  
	//Especificamos el rol del ESP-NOW (0=OCIOSO, 1=MAESTRO, 2=ESCLAVO y 3=MAESTRO+ESCLAVO)
	esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
	//Emparejamos con el otro ESP
	esp_now_add_peer(macPasarela, ESP_NOW_ROLE_CONTROLLER, WIFI_CHANNEL, key, 0);
	//Especificamos las claves para enviar la información de manera encriptada.
  esp_now_set_peer_key(macPasarela, key, 16);

  //Comprueba si un paquete de datos enviado ha sido recibido correctamente por un par
	esp_now_register_send_cb([](uint8_t* mac, uint8_t sendStatus) {
		Serial.printf("Mensaje enviado, estado (0=OK, 1=ERROR) = %i\n", sendStatus);
	});
  
	//Si llega un mensaje por ESP-NOW a esta ESP, cambiamos el valor del LED.
	esp_now_register_recv_cb(OnRecv);
}

void loop() {
  
  //Un condicional para controlar cada cuanto tiempo se envía información del sensor.
  if(countTime<millis()){

    //Encedemos el LED
    digitalWrite(LEDCheck, HIGH);
    
    //Esperamos a que el sensor esté preparado para poder leerlo.
    delay(dht.getMinimumSamplingPeriod());
    
    //Leemos el sensor.
    float t= dht.getTemperature();
    t = (isnan(t))? -255 : t;
    float h= dht.getHumidity();
    h = (isnan(h))? -255 : h;
    
    //Creamos y mostramos el JSON que se va a enviar.
    sprintf(mensaje,"orchard/espnow/datos|{\"hum\":%g,\"temp\":%g,\"status\":\"%s\"}", h, t, dht.getStatusString());
    Serial.printf("Mensaje: %s\n",mensaje);
    
    //Enviamos el JSON por ESP_NOW
    esp_now_send(macPasarela, (uint8_t *) mensaje, strlen(mensaje)+1);
    
    //Apagamos el LED
    digitalWrite(LEDCheck, LOW); 

    //Calculamos el tiempo para envíar la siguiente información.
    countTime = millis() + frecuenciaActualizacion*1000;
	
  	//Si da la casualidad de que, después de enviar los datos del sensor, también va a enviar el mensaje de que está vivo, lo retrasamos un segundo para que no haya interferencias.
  	if(countAliveTime!=0 && countAliveTime<millis()){ countAliveTime=millis()+1000; }
  }
  
  //Un condicional para controlar cada cuanto tiempo se envía un keepalive.
  if(countAliveTime<millis()){

    //Encedemos el LED
    digitalWrite(LEDCheck, HIGH);
    
    //Creamos y mostramos el JSON que se va a enviar.
    sprintf(mensaje,"orchard/espnow/estado|{\"status\":\"alive\",\"freq\":\"%i\"}", frecuenciaActualizacion);
    Serial.printf("Mensaje: %s\n",mensaje);
    
    //Enviamos el JSON por ESP_NOW
    esp_now_send(macPasarela, (uint8_t *) mensaje, strlen(mensaje)+1);

    //Apagamos el LED
    digitalWrite(LEDCheck, LOW);

    //Calculamos el tiempo para envíar la siguiente información.
    countAliveTime = millis() + ALIVE_SECS*1000;
  }
  
}

// callback function executed when data is received
void OnRecv(uint8_t *mac_addr,  uint8_t *data, uint8_t data_len) {

  Serial.printf("\r\nReceived\t%d Bytes\t%d", data_len, data[0]);
}
