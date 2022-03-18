#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <MQTT.h>
#include "myDefines.h"
#include "build_defs.h"


/**********************************************************************
   VARS
***********************************************************************/
WiFiClientSecure espClient;
MQTTClient mqtt_client;
SoftwareSerial swSer(14, 12, false);

// Certificados TLS: Certificate Authority
const char root_ca[] PROGMEM = R"EOF(-----BEGIN CERTIFICATE-----
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

char espID[32];

//Cadenas para topics e ID.

char topic_PUB[256];
char mensaje_mqtt[512];
char JSON_serie[256];
String deviceMac;

const char* mqtt_user = NULL;
const char* mqtt_pass = NULL;

//Variable para controlar el tiempo.
unsigned long ultimo_mensaje = 0;
/**********************************************************************
  ARDUINO SETUP & MAIN LOOP
***********************************************************************/

void setup()
{
  Serial.begin(115200);
  connectToNetwork();
  mqtt_client.begin(MQTT_SERVER, MQTT_PORT, espClient);
  mqtt_client.onMessage(callback);

  swSer.begin(SERIAL_BAUD_RATE);
}

void loop() {
  unsigned long rightNow = millis();
  if (!mqtt_client.connected()) {
    reconnect();
  }
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
        ultimo_mensaje = rightNow;
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
        ultimo_mensaje = rightNow;
      }
    }
  }
  //Si el LED está encendido porque se ha enviado un mensaje por el protocolo serie y hace más de 0,2s que está encedido, se apaga.
  if (digitalRead(LEDCheck) == HIGH && rightNow - ultimo_mensaje >= 200) {
    digitalWrite(LEDCheck, LOW);
  }
}

/***********************************************************************
  OPERATIONAL FUNCTIONS
***********************************************************************/
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToNetwork() {
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
  // DESCOMENTAR CUANDO SE DESCOMENTE WIFISECURE!!
  espClient.setCACert(root_ca);
  espClient.setInsecure();
  espClient.setTimeout(12);
}

void reconnect() {
  String clientId = "ESP32Client-";
  uint32_t chipID = ESP.getEfuseMac();
  clientId += String(chipID);
  clientId.toCharArray(espID, 32);
  String topic_string_connect = ("orchard/" + TYPE_NODE + "/" + clientId + "/connection");      // Select topic by ESP ID : connection
  const char* conexion_topic = topic_string_connect.c_str();
  String topic_string_led = ("orchard/" + TYPE_NODE + "/" + clientId + "/led");      // Select topic by ESP ID : led intensity
  const char* led_topic = topic_string_led.c_str();
  String topic_string_baudRate = ("orchard/" + TYPE_NODE + "/" + clientId + "/baudrate");      // Select topic by ESP ID : Serial baud rate
  const char* baudRate_topic = topic_string_baudRate.c_str();

  // BUCLE DE CHECKING DE CONEXIÓN AL SERVICIO MQTT
  while (!mqtt_client.connected()) {
    Serial.print("Intentando conectarse a MQTT...");

    // Generación del nombre del cliente en función de la dirección MAC y los ultimos 8 bits del contador temporal

    Serial.print("Conectando a ");
    Serial.print(MQTT_SERVER);
    Serial.print(" como ");
    Serial.println(clientId);

    // Intentando conectar
    if (mqtt_client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("conectado");
      // Check IOTfingerprint

      // Nos suscribimos a los siguientes topics
      mqtt_client.subscribe(led_topic);
      mqtt_client.subscribe(baudRate_topic);
      // Publicamos el estado de la conexion en el topic
      mqtt_client.publish(conexion_topic, "Online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.println(" try again in 5 seconds");
      // Espera 5s antes de reintentar conexión
      delay(5000);
    }
  }
}

void callback(String &topic, String &payload)
{
  Serial.write("$$");
  Serial.println(payload.length());
  Serial.println(payload);
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
  mqtt_client.publish(topic_PUB, mensaje_mqtt, true);

}

inline String byte2HEX (byte data)
{
  return (String(data, HEX).length() == 1) ? String("0") + String(data, HEX) : String(data, HEX);
}
/***********************************************************************
   INTERRUPT FUNCTIONS
***********************************************************************/



/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/
