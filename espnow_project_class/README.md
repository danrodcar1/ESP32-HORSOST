Proyecto ESP-NOW Autopairing
============================

Este proyecto está basado en el trabajo de fin de máster del máster en Ingeniería Mecatrónica.

Las funciones que se encargaban, por ejemplo, de la conexión por ESP-NOW al dispositivo que hacía la función de pasarela, así como el proceso de configuración o ajustes previos y el proceso de actualización OTA, se ha incluido en una clase dentro de la librería “AUTOpairing.h” dentro de core/inc. Por otro lado, el funcionamiento del convertidor ADC se ha modelado en la librería “ADConeshot.h”. La estructura es más sencilla y permite modular el código en diferentes clases que permiten modelar el comportamiento del dispositivo de una manera más eficiente.

•	La librería “AUTOpairing.h” se ha diseñado para albergar una clase que componen el funcionamiento del auto emparejamiento del dispositivo que se pretende conectar a la pasarela y las funciones que modelan la configuración y conexión al servidor que alojan el fichero para realizar la actualización FOTA. 

•	La librería “ADConeshot.h” modela el comportamiento del convertidor analógico-digital. Permite ser configurado, seleccionar el canal y varios parámetros en función de las necesidades que sean requeridas. 
Este modelado en clases permite, en definitiva, modular y realizar un código más sencillo, cuya funcionalidad se reduce exclusivamente a lo que se pretende realizar. El código funcional está escondido al usuario, de tal modo que este solamente se preocupa de configurar los diferentes parámetros funcionales que necesite.


*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*
