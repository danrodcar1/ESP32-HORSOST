
# Pasarela de comunicación ESP-NOW a MQTT

1. **EspNowtoMQTTGateway**: Flujo que implementa una pasarela sencilla, basada en ESP8266, que comunica un dispositivo (peer) desde ESP-NOW a MQTT.
     - El código permite la comunicación unidireccional peer a MQTT, sin posibilidad de que el peer pueda recibir datos o mensajes por MQTT.

3. **SerialtoMQTTGateway**: Protocolo de comunicación ESP-NOW a MQTT usando comunicación serie. 
     - Esta rutina si permite la comunicación peer a MQTT y viceversa. El protocolo de comunicación que se debe de seguir está en la carpeta.
