GRUPO 16

Somos Adrià Sala, Lucas Fopiani y Miguel Alfonso Sanz.

En este proyecto hemos usado las herramientas de Arduino para hacer una versión muy simplificada de lo que implica un proyecto de control de un satélite. Una placa de Arduino tiene la función de satélite, con sensores, motores, etc., y es capaz de comunicarse con otra placa, la cual representa ser la estación de tierra. La estación de tierra es la que se encarga de monitorizar y verificar el correcto funcionamiento del satélite, y todo esto es posible visualizarlo a través de una interfaz gráfica construida en Python. 

Este sistema cuenta con las funcionalidades más fundamentales: es capaz de poder comunicarse inalámbricamente, implementa sistemas para la detección de errores en las medidas de los sensores, fallos de conexión, problemas en la comunicación, alertas que notifican estos sucesos, y las respuestas adecuadas en cualquier caso. Además, se puede interactuar y modificar algunas características del satélite.  


CARACTERÍSTICAS

Comunicación inalámbrica mediante LoRa entre dos Arduinos.
Recepción y envío confiable de datos entre satélite y estación tierra.
Visualización de datos en Python con gráficas y diagramas tipo radar.
Estructura modular que facilita agregar nuevos sensores o métricas.


TECNOLOGÍAS UTILIZADAS

Hardware: Arduino UNO, módulos LoRa
Software Arduino: IDE Arduino con librerías LoRa y DHT11
Software Python:
  matplotlib para gráficas
  tkinter para la interfaz
  numpy para manejo de datos
