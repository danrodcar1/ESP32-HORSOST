
# Pasarela de comunicación ESP-NOW a MQTT

1. **EspNowtoMQTTGateway**: Flujo que implementa una pasarela sencilla, basada en ESP8266, que comunica un dispositivo (peer) desde ESP-NOW a MQTT.
     - El código permite la comunicación unidireccional peer a MQTT, sin posibilidad de que el peer pueda recibir datos o mensajes por MQTT.
     - El usuario debe de respetar el envío de mensajes, usando para ello el siguiente formato: **topic|{message}**. El código separará el topic y el mensaje y publicará dicho mensaje en el topic correspondiente.

3. **SerialtoMQTTGateway**: Protocolo de comunicación ESP-NOW a MQTT usando comunicación serie. 
     - Esta rutina si permite la comunicación peer a MQTT y viceversa. El protocolo de comunicación que se debe de seguir está en la carpeta.
     - El usuario debe de respetar el envío de mensajes, usando para ello el siguiente formato: **topic|{message}**. El código separará el topic y el mensaje y publicará dicho mensaje en el topic correspondiente.
     - Se incluye una presentación del protocolo de ***RECOMENDADA LECTURA*** para que se entienda el proceso de envío y recepción de datos.
