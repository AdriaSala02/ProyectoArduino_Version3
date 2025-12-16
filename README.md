GRUPO 16


Este proyecto implementa un sistema de comunicación inalámbrica utilizando tecnología LoRa entre dos Arduinos, donde uno actúa como satélite y otro como estación terrestre, y una interfaz en Python que permite visualizar en tiempo real los datos captados por el satélite mediante gráficas y radares.

------------------------------------------------------------------------------------------------------------------------------------------------------

DESCRIPCIÓN

El objetivo de este proyecto es demostrar la comunicación de datos entre dispositivos remotos mediante LoRa y su posterior visualización en una interfaz amigable.

Arduino Satélite: Captura datos de sensores y los envía a través de LoRa.

Arduino Estación Tierra: Recibe los datos enviados por el satélite y los retransmite a la computadora mediante USB.

Interfaz Python: Procesa los datos recibidos desde la estación terrestre y los muestra en gráficas y radares en tiempo real, permitiendo un análisis visual intuitivo.

------------------------------------------------------------------------------------------------------------------------------------------------------

CARACTERÍSTICAS

Comunicación inalámbrica mediante LoRa entre dos Arduinos.

Recepción y envío confiable de datos entre satélite y estación tierra.

Visualización de datos en Python con gráficas y diagramas tipo radar.

Estructura modular que facilita agregar nuevos sensores o métricas.

------------------------------------------------------------------------------------------------------------------------------------------------------

TECNOLOGÍAS UTILIZADAS

Hardware: Arduino UNO/Nano (o similar), módulos LoRa (SX1278, RFM95, etc.)

Software Arduino: IDE Arduino con librerías LoRa

Software Python:

matplotlib para gráficas

tkinter o PyQt para la interfaz (dependiendo de implementación)

pandas o numpy para manejo de datos

------------------------------------------------------------------------------------------------------------------------------------------------------

CONTRIBUCIÓN

Este proyecto fue desarrollado por Miguel, Lucas y Adrià Sala.
