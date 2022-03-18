/**
   Recibe por serie un mensaje que también contiene la MAC del emisor original y lo publica en MQTT
   Publica en el topic infind/espnow/MAC en un servidor local 192.168.1.101
*/

//Librerias.
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <MQTT.h>
#include <WiFiClientSecure.h>

//Creamos el cliente MQTT y la conexión segura.
BearSSL::WiFiClientSecure net;
MQTTClient mqtt_client;

#define BAUD_RATE 115200

// GPIOs.
#define LEDCheck 2

//Configuramos el protocolo serie.
SoftwareSerial swSer(14, 12, false);

//Certificado para el TLS de MQTT.
static const char digicert[] PROGMEM = R"EOF(
    -----BEGIN CERTIFICATE-----
MIIEPTCCAyWgAwIBAgIUQ37GWo/JYvONKEe2AjFsoEFWi50wDQYJKoZIhvcNAQEL
BQAwga0xCzAJBgNVBAYTAkVTMQ8wDQYDVQQIDAZNYWxhZ2ExDzANBgNVBAcMBk1h
bGFnYTEeMBwGA1UECgwVVW5pdmVyc2lkYWQgZGUgTWFsYWdhMSMwIQYDVQQLDBpJ
bmdlbmllcmlhIGRlIGNvbXB1dGFkb3JlczEVMBMGA1UEAwwMc2VydmVyIE1RVFQg
MSAwHgYJKoZIhvcNAQkBFhFjYXJyaW9uMDI0QHVtYS5lczAeFw0yMjAzMDgxMDUw
MDdaFw0yNzAzMDgxMDUwMDdaMIGtMQswCQYDVQQGEwJFUzEPMA0GA1UECAwGTWFs
YWdhMQ8wDQYDVQQHDAZNYWxhZ2ExHjAcBgNVBAoMFVVuaXZlcnNpZGFkIGRlIE1h
bGFnYTEjMCEGA1UECwwaSW5nZW5pZXJpYSBkZSBjb21wdXRhZG9yZXMxFTATBgNV
BAMMDHNlcnZlciBNUVRUIDEgMB4GCSqGSIb3DQEJARYRY2FycmlvbjAyNEB1bWEu
ZXMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQC1tSmdtwK/TVAo8QWF
lPcQmXXqscLe4FT9lt0aB74C/VlD1q1P27YOeGbijTL3TEiVlDakW6wPCtpBTVbA
D049XfNObu/wroyKoO6M/OELiB29LQ71CQKBaqeuNUVi349EVwcnM82lY/XWi8Uu
wdyb6Odr7q4awhonIQxoaQ2djlC1pRKeL7f4yPGSWH4Xq4PgerQRLZY/DRoOEjY3
O+WXa+TT+R3/nMQ9yheBB5w6+COmrKZTWtV2FyjRRS1NKL0SlcmDZXcP90RbY1C4
J/Pu+7ex4Yan9LNbWTb/4mu08cGsEK99VjbP30egANhBtO7lLkd/1ZXYgVMnJj50
JIhZAgMBAAGjUzBRMB0GA1UdDgQWBBQPvoFoLZkIlXp1DjnL2dm3ZZQW5jAfBgNV
HSMEGDAWgBQPvoFoLZkIlXp1DjnL2dm3ZZQW5jAPBgNVHRMBAf8EBTADAQH/MA0G
CSqGSIb3DQEBCwUAA4IBAQBIlusg9lptwaRZPj1TezdM9qhKWqUpx7qW/62TeMc9
C3EpHoiuf3vG6GNM7Bfi9c+Jzm3/NoqxS+sXWTanVn/ytRvkxNJEk6HS7YlNmVp3
P5Y3tKi+NLjl02Rereu4uRK13yzooM3JSL/07EtvXnuSUjQQyEwLhTeVotbEY68Q
D4gcy0DW/VnHlobwEYfY2JlcXGpmrg7IMuHEjD+ANszYanfarp5sU+EOGe4CwDDY
FcwAWN5U9FDM03BAHfWqshDb3ZDRMZK08NRlmMUVDejDer8iEaoAHWDYnQohWJ0L
k6h96o223t/xcDX1gugXRSXljJ5hdaN2e30o5aS1gkSi
-----END CERTIFICATE-----
    )EOF";

//Configuramos variables para conexión con WiFi y MQTT.
const char* ssid = "huerticawifi";
const char* password = "4cc3sshu3rt1c4";
const char* mqtt_server = "10.10.10.10";
const char* mqtt_user = NULL;
const char* mqtt_pass = NULL;

//Cadenas para topics e ID.
char ID_PLACA[16];
char topic_PUB[256];
char mensaje_mqtt[512];
char JSON_serie[256];
String deviceMac;

//Variable para controlar el aviso de "Esperando mensajes ESP-NOW".
unsigned long heartBeat = 0;
//Variable para controlar el tiempo.
unsigned long ultimo_mensaje = 0;

//Método para conectar al WiFi.
void conecta_wifi() {
  Serial.printf("\nConectando a %s:\n", ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  Serial.printf("\nWiFi conectado, dirección IP: %s\n", WiFi.localIP().toString().c_str());
}

//Método para conectar al MQTT.
void conecta_mqtt() {
  //Hasta que no se conecte a MQTT...
  while (!mqtt_client.connected()) {

    Serial.print("Conectando al MQTT...");

    //Si se conecta
    if (mqtt_client.connect(ID_PLACA, mqtt_user, mqtt_pass)) {

      Serial.printf(" conectado a broker: %s\n", mqtt_server);

      //Suscribirse al topic para controlar el LED.
      mqtt_client.subscribe("infind/espnow/led");
      Serial.println("Suscrito a infind/espnow/led");

      //Suscribirse al topic para controlar la frecuencia de actualización.
      mqtt_client.subscribe("infind/espnow/frecuencia");
      Serial.println("Suscrito a infind/espnow/frecuencia");

      //Si no se conecta
    } else {
      //Reintento.
      Serial.println(" fallido,  se intenta de nuevo en 5s");
      delay(5000);
    }
  }
}

//Método que devuelve 2 caracteres HEX para un byte.
inline String byte2HEX (byte data)
{
  return (String(data, HEX).length() == 1) ? String("0") + String(data, HEX) : String(data, HEX);
}

//Método que se encarga de recoger los datos que llegan del protocolo serie y se envía por MQTT al broker.
inline void readSerial() {

  //Obtenemos la dirección MAC.
  deviceMac = "";
  while (swSer.available() < 6) {
    delay(1);
  }
  for (int i = 0; i < 6; i++) deviceMac += byte2HEX(swSer.read());
  for (auto & c : deviceMac) c = toupper(c);

  //Obtenemos la longitud del JSON que vamos a recibir.
  while (swSer.available() < 1) {
    delay(1);
  }
  byte len =  swSer.read();

  //Obtenemos el JSON
  while (swSer.available() < len) {
    delay(1);
  }
  swSer.readBytes((char*)&JSON_serie, len);
  JSON_serie[len] = '\0'; //fin de cadena, por si acaso

  //Preparamos el topic y el payload del MQTT. Publicamos el mensaje.
  sprintf(mensaje_mqtt, "{\"mac\":\"%s\",\"mensaje\":%s}", deviceMac.c_str(), JSON_serie);
  mqtt_client.publish(topic_PUB, mensaje_mqtt);
}

//Método que se encarga de recoger lo que llega del MQTT y se envía por el protocolo serie.
void OnMqttReceived(String &topic, String &payload)
{
  Serial.write("$$");
  Serial.println(payload.length());
  Serial.println(payload);
}

void setup() {

  Serial.begin(115200);
  Serial.println();

  //Inicializamos el LED y el ID de la placa.
  pinMode(LEDCheck, OUTPUT);
  digitalWrite(LEDCheck, LOW);
  sprintf(ID_PLACA, "ESP_%d", ESP.getChipId());

  //Conectar a WiFi
  conecta_wifi();

  //Especificamos el certificado
  BearSSL::X509List cert(digicert);
  net.setTrustAnchors(&cert);
  net.setInsecure();

  //Configuramos MQTT y protoloco serie
  mqtt_client.begin(mqtt_server, 8883, net);
  mqtt_client.onMessage(OnMqttReceived);

  swSer.begin(BAUD_RATE);
}

void loop() {

  //Obtenemos el momento actual.
  unsigned long ahora = millis();

  //Si no está conectado al MQTT, nos conectamos.
  if (!mqtt_client.connected()) conecta_mqtt();
  //Esta llamada para que la librería recupere el controlz
  mqtt_client.loop();

  //Si tenemos algún mensaje por serie, entramos en el bucle.
  while (swSer.available()) {

    char dato = swSer.read();

    //Comprobamos si viene primero el icono '$'
    if ( dato == '$' ) {

      //En caso de venir ese icono, comprobamos que el segundo sera también '$'. En caso de ser, es un JSON con información del sensor y vamos a proceder a leerlo.
      while ( ! swSer.available() )  delay(1);

      char dato2 = swSer.read();

      if ( dato2 == '$' ) {
        //Enciende el led al enviar mensaje
        digitalWrite(LEDCheck, HIGH);
        //Establecemos el topic
        sprintf(topic_PUB, "infind/espnow/%s/datos", deviceMac.c_str());
        //Leemos los datos.
        readSerial();
        ultimo_mensaje = ahora;
      }
    }

    //Comprobamos si viene primero el icono %'
    if ( dato == '%' ) {

      //En caso de venir ese icono, comprobamos que el segundo sera también '%'. En caso de ser, es un JSON con información del keepalive. Procedemos a leerlo.
      while ( ! swSer.available() )  delay(1);

      char dato2 = swSer.read();

      if ( dato2 == '%' ) {
        //Enciende el led al enviar mensaje
        digitalWrite(LEDCheck, HIGH);
        //Establecemos el topic
        sprintf(topic_PUB, "infind/espnow/%s/status", deviceMac.c_str());
        //Leemos los datos.
        readSerial();
        ultimo_mensaje = ahora;
      }
    }
  }

  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LEDCheck) == HIGH && ahora - ultimo_mensaje >= 200) {
    digitalWrite(LEDCheck, LOW);
  }
}
