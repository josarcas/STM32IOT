# STM32IOT
Desarrollo de sistema operativo con GUI, basado en la plataforma STM32 IOT NODE


Uso de la plataforma B-L475E-IOT01A desarrollada por STM32, para el desarrollo de un sistema operativo en tiempo real, contiene una interfaz grafica sencilla e interección
con internet a traves del modulo ism43362-m3g-l44, la librería "wifi_driver" contiene funciones para conexion a un modem wifi, a un broker MQTT, a servidores SMTP, POP, SNTP, y spi
para obtener el tiempo (http get). La interfaz grafica esta a cargo de una pantalla TFT 2.4 inch, tambien contiene comunicacion via SPI con una tarjeta SD para guardar y recuperar
datos.
