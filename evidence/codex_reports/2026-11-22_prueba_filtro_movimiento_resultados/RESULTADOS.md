# Prueba experimental del filtro durante movimiento físico de J1

Fecha: 22 de noviembre de 2026, Asia/Manila. Objetivo: registrar y evaluar consistencia de la segmentación durante una trayectoria real. El operador confirmó que estaba autorizado, que el área estaba despejada y que la parada de emergencia estaba accesible; pidió un movimiento de +10° en J1 a 5°/s. La fuente de proximidad era una esfera virtual PARK, que no detecta obstáculos físicos.

## Procedimiento y trazabilidad

Se ensayó primero el comando en Gazebo aislado en ROS_DOMAIN_ID 77: +10° en J1 durante 2 s. El supervisor simulado aceptó con estado WARNING y Gazebo devolvió SUCCEEDED; la postura simulada era diferente de la física. Se cerró la simulación antes de iniciar la grabación física. Una evaluación cinemática independiente de la postura física observada y la esfera PARK indicó separación mínima aproximada de 0,7726 m durante +10°, pero no modela el entorno real.

La primera ejecución del cliente físico no publicó comandos porque una comprobación interpretaba el nivel de diagnóstico ROS (`b'\x00'`) como entero. Se corrigió para comparar con `DiagnosticStatus.OK`. En la siguiente ejecución se verificaron diagnóstico sin fallo, conexión, armado, velocidades publicadas cercanas a cero, `READY`, proximidad geométricamente válida y `ALLOW`, sin objetivo o JOG activos. Después se publicó una única referencia articular con ID `codex_filtro_j1_10deg_20261122`, desplazamiento de J1 +10° y duración nominal de 2 s. Los otros cinco objetivos se tomaron de la misma lectura articular.

El supervisor emitió `ALLOW/CLEAR`; el adaptador emitió `ACCEPTED` y `SUCCEEDED`. El registro incluye `command_decision`, `supervised_command`, `execution_trajectory`, TF, estados, tres clases, imagen, nube original y diagnósticos. Aunque el registrador estaba suscrito, el tópico `candidate_command` contiene 0 mensajes en el bag; los datos de decisión, comando supervisado y ejecución sí quedaron registrados. La aceptación también consta en la salida del cliente durante la sesión. Esto limita la trazabilidad del mensaje original y no debe ocultarse.

Bag: `../2026-11-22_filtro_j1_movimiento_10grados/` (44,383 s; 7,5 GiB). `antes.png`, `durante.png` y `despues.png` son imágenes extraídas del bag. Contienen la escena del laboratorio y no son máscaras de referencia.

## Movimiento medido

De 888 mensajes de `/joint_states`: mediana inicial J1 2,98756021 rad, mediana final 3,16210379 rad; cambio aproximado +10,0006°. El inicio y final de movimiento estimados mediante diferencias de posición superiores a 0,1°/s fueron `1795320246.567717` y `1795320248.767693` segundos Unix. La máxima diferencia finita observada entre muestras consecutivas fue aproximadamente **6,02°/s**, superior a los 5°/s nominales: es una estimación sensible a cuantización, marcas y frecuencia de muestreo, no una medición certificada de pico. Debe investigarse y no se repitió ni amplió el movimiento. `SUCCEEDED` significa que el adaptador declaró el objetivo dentro de tolerancia.

Tras la trayectoria, el diagnóstico consultado indicaba conectado, armado, HOLD, sin objetivo ni JOG activos, `fault=false` y salida de comandos 0 Hz. El brazo quedó armado; no se ejecutaron otras trayectorias en esta prueba.

## Comparación de nubes durante el movimiento

Dentro del intervalo temporal estimado de movimiento se hallaron 40 marcas comunes a la nube original y las tres salidas. El análisis `analizar_movimiento.py` procesó las 40:

- **1.189.303 registros de puntos válidos**, tras rango radial y transformación experimental.
- En **40/40** capturas, la unión multiconjunto de brazo, incierto y resto coincidió byte por byte con la entrada transformada.
- **0 diferencias** de etiqueta entre la GPU y una referencia CPU con las mismas mallas muestreadas, cápsulas y TF al sello en esas 40 capturas.
- La clase resto tuvo como máximo **2 puntos** en una de esas capturas. Sin verdad de terreno no puede afirmarse que sean residuos del brazo o ruido.

La comparación prueba consistencia de implementación para esta trayectoria y este modelo, no exactitud semántica independiente. La superposición cámara–brazo no está calibrada físicamente y ambas implementaciones pueden compartir el mismo error. No había un objeto de referencia visible y etiquetado cerca del robot, por lo que el ensayo no permite medir conservación de obstáculos, precisión o recall. Tampoco prueba otras articulaciones, velocidades, trayectorias, oclusiones o movimientos cartesianos.

Los intervalos se eligen con marcas de estado publicadas; no hay reloj externo independiente. La lectura de GPU/CPU utiliza los datos y código archivados en la auditoría técnica anterior. Las capturas comunes son una intersección de marcas registradas; no representan necesariamente cada nube producida por la cámara.

## Archivos

- `resultados_dinamicos.json`: resultados por marca de tiempo, cantidades y comparación.
- `analizar_movimiento.py`: análisis reproducible desde la raíz del proyecto con dependencias de ROS/NumPy/SciPy. Usa la matriz, puntos de malla y cápsulas archivados en la auditoría de la sesión.
- `codex_analyze_move.py`: inspección de decisiones y posiciones del bag.
- `codex_prueba_j1_fisico.py`: cliente de la prueba; **contiene un publicador de movimiento y no debe ejecutarse como análisis offline**.
- `codex_sim_j1_5dps.py`: prueba aislada de simulación; también publica una orden y requiere dominio simulado aislado.
- `SHA256SUMS.txt`: identificación de los archivos indicados, no una certificación de exactitud física.
