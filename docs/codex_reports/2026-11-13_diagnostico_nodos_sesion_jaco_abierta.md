# Diagnóstico pasivo de la sesión JACO abierta

Fecha: 2026-11-13, Asia/Manila.
Solicitud: analizar nodos, captura RViz y cómo resolver el comportamiento.
Modo: observación. No se cambiaron código, parámetros, controles ni marcadores
de la sesión; no se enviaron órdenes de movimiento ni se detuvo el launch.
Este archivo registra el diagnóstico, no implementa las recomendaciones.

## Sesión identificada

Launch PID 32814: `thesis_system.launch.py`, `headless=false`, GUI y RViz
habilitados, salida simulada habilitada y fuente `gazebo`.
Dominio ROS 81; partición Ignition `jaco_prueba`. Sin bridge físico detectado.

La consulta CLI inicial de nodos, con `--no-daemon --spin-time 2`, solo descubrió
parte del grafo. El observador posterior con 12 s de descubrimiento encontró
todos los componentes esperados y ningún duplicado de esos componentes:
RSP, puentes de reloj/pose, RViz, visualizador de cápsulas, proximidad,
supervisor, adaptadores articular/cartesianos, horizonte, GUI, disponibilidad,
ros2_control, controller_manager y sus dos controladores. Hay además listeners
TF auxiliares. La lista inicial parcial no demostraba caída de nodos Python.

## Datos vivos

Primer observador, 12 s:

- Proximidad: 193 mensajes; 146 ALLOW con geometría válida y 47 STOP por
  TF_UNAVAILABLE con geometría inválida.
- Distancia de la última medida válida: 0.3672573158309574 m. La captura aportada
  por el usuario muestra aproximadamente 0.395 m. Ambos valores superan el
  umbral WARNING de 0.30 m.
- Coherencia FK-TF: 96 mensajes VALID, error 0 m, tolerancia 0.005 m.
- Disponibilidad: 11 READY y 1 WAITING por TF no disponible.
- Horizonte: 120 mensajes; etiquetas alternaron ALLOW y STOP, con fuente de
  velocidad cero, postura constante y sin intención vigente.
- No se recibieron nuevas decisiones de comando, ejecuciones ni controles
  activos en esa ventana. No se afirma que nunca hubiera comandos anteriores.
- Factor de tiempo real observado: aproximadamente 0.801.

Segundo observador, aproximadamente 10 s útiles tras calentamiento:

| Dato | Valor medido |
| --- | ---: |
| Parámetro RSP publish_frequency | 20 Hz |
| TF de robot, tiempo simulado | 20.000 Hz |
| TF de robot, tiempo de pared | 16.773 Hz |
| Pose del obstáculo, tiempo simulado | 29.412 Hz |
| Pose del obstáculo, tiempo de pared | 24.661 Hz |
| Espera mediana hasta TF que abarca el sello del obstáculo | 30.012 ms |
| Espera máxima observada | 61.158 ms |
| Muestras con espera superior a 40 ms | 80 de 246 |

Se consultó el parámetro mediante GetParameters de solo lectura. Los intervalos
se midieron pasivamente a partir de mensajes de robot y obstáculo; no se
intervino en sus frecuencias. Son dos ventanas diagnósticas distintas, por lo
que no se relaciona cada espera con un STOP individual de la primera ventana.

El monitor tiene un presupuesto total de espera TF de 40 ms y pide el sello
exacto del obstáculo. TF a 20 Hz implica hasta 50 ms simulados entre muestras;
con factor de tiempo real cercano a 0.8 eso alcanza unos 62 ms de pared. La
disponibilidad temporal no alcanza siempre ese presupuesto, aunque la geometría
sea correcta cuando el conjunto de transformaciones llega completo.

CPU: 16 procesadores lógicos, carga de un minuto aproximadamente 15.26 y presión
de CPU observada. Memoria disponible aproximadamente 23 GiB; no se encontró
evidencia de falta de RAM. El mundo monobrazo publica reloj de pasos de 1 ms.

## Interpretación de la captura

El rojo corresponde a capas de volumen/seguridad, no necesariamente al material
del robot. El horizonte usa rojo ante STOP y azul ante ALLOW. La sesión alterna
por TF_UNAVAILABLE, lo que explica el rojo intermitente junto a una línea verde
de una medida válida recibida en otro momento.

Hay dos defectos adicionales de presentación confirmados por revisión de código:

1. CapsuleVisualizer guarda proximidad y predicción sin caducidad ni validación
   de `geometry_valid`, y vuelve a publicar sus marcadores con fecha actual.
   Puede mostrar el cero de fallo TF como medida y conservar predicción
   histórica. En el sondeo actual no se vieron marcadores de predicción: se
   registra como defecto del código, no como causa histórica demostrada aquí.
2. Se superponen cápsulas actuales y volumen de horizonte. La línea verde une
   eje de cápsula y centro de esfera, aunque el texto resta los radios y expresa
   separación entre superficies. La diferencia visual no es un error aritmético.

La GUI procesa un solo callback por temporizador de 10 ms. Su límite teórico de
unos 100 callbacks/s puede ser insuficiente para todas sus suscripciones. Es un
riesgo confirmado en el diseño; no se inspeccionó la memoria interna de la GUI
ni se demostró la edad de sus colas durante esta observación.

## Correcciones propuestas, pendientes de implementación

1. Configurar RSP a 50–100 Hz, preferiblemente 100 Hz para el feedback de 100 Hz,
   y repetir la prueba con Gazebo, RViz y GUI abiertos. Conservar la consulta al
   sello exacto, los umbrales y el presupuesto TF; no sustituir por último TF.
   Evaluar carga de CPU y pérdidas después del cambio.
2. En RViz, distinguir dato TF desconocido de proximidad geométrica; caducar y
   retirar datos históricos de cápsulas/predicción, sin modificar el STOP real
   del supervisor. Simplificar capas y texto iniciales para evitar solapamiento.
3. Drenar callbacks de la GUI en un lote con presupuesto de tiempo y usar colas
   acotadas para telemetría. Validar foco, JOG y capacidad de respuesta de la
   ventana mediante las regresiones existentes y pruebas de carga.

Comprobación visual reversible para el operador: desmarcar HorizonVolume en
RViz permite ver el robot y las cápsulas sin esa superposición. Esto solo cambia
la visualización y no resuelve ni altera la seguridad temporal.
