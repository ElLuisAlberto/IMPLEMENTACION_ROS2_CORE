# Histórico de mejoras: D435i y separación visual del JACO2

Fecha del registro: 22 de noviembre de 2026. Alcance: las mejoras y pruebas de la cadena de observación y segmentación tratadas en la conversación disponible, junto con antecedentes documentales relacionados. Este documento es una reconstrucción; no constituye un registro completo de cada acción anterior ni atribuye automáticamente al mismo autor todos los cambios encontrados.

## Procedencia y estado de la evidencia

- **Código inspeccionado:** archivos actuales del proyecto y prototipos de `/tmp`. Su presencia demuestra su contenido al momento de esta revisión, no por sí sola que hayan sido ejecutados o validados en todas las condiciones.
- **Contexto de conversación:** las cifras iniciales y la descripción de las primeras visualizaciones provienen de la respuesta anterior compartida por el usuario. No se dispone aquí de las capturas originales ni de los registros crudos de esas cifras.
- **Mediciones observadas:** las mediciones de frecuencia, antigüedad y comparación sintética indicadas abajo se ejecutaron durante la continuación de esta conversación. Son muestras puntuales, no ensayos de larga duración.
- **Control de versiones:** la referencia Git inspeccionada es `cce8191`. Hay modificaciones previas sin confirmar y archivos nuevos, entre ellos cambios de percepción; su autoría y cronología exactas no pueden deducirse solo de `git status`. No se han revertido ni modificado para este informe.
- **Temporalidad:** los prototipos y varios datos están en `/tmp`, ubicación que puede limpiarse o cambiar; este informe no los preserva. Las horas asociadas a archivos temporales se usan solo para ordenar indicios, no como sellos de auditoría confiables. La fecha de este informe usa la fecha de la sesión proporcionada por el usuario.

## Cronología reconstruida

### 1. Antecedentes de percepción y calibración

Existían trabajos anteriores sobre residuales de malla, máscaras del brazo, alineación y ChArUco. El diagnóstico [fase 8O](2026-10-22_fase8o_diagnostico_mesh_arm_residual.md) concluyó que los datos de entonces no permitían atribuir con rigor los residuos a la extrínseca global o a discrepancias locales del modelo. La [fase 8U](2026-10-22_fase8u_preparacion_charuco_temporal.md) preparó un tablero y una herramienta observacional, sin afirmar una calibración física aceptada en esa fase. Sus conclusiones se refieren a aquellas sesiones, no prueban el estado de la sesión actual.

En los cambios actuales de `tools/perception/charuco_observational_once.py` se observa una conversión explícita entre el marco del tablero de OpenCV antiguo y el contrato de origen y ejes del tablero; `src/thesis_jaco_perception/test/test_charuco_observational_offline.py` contiene pruebas sintéticas asociadas. Se registran como cambios presentes, sin adjudicarles una ejecución o resultado de prueba en esta auditoría.

### 2. Adquisición y superposición experimental

En el árbol de trabajo actual, `thesis_perception/launch/d435i_standalone.launch.py` configura profundidad de 848×480 a 30 Hz y color de 1280×720 a 30 Hz, High Density, potencia del emisor 150 y exposición manual de 10.000 µs. El lanzamiento deja desactivados por defecto los filtros de decimación, espacial y temporal, y el voxelizado; se pueden activar para experimentos. Frente a la referencia Git también incorpora selección de NVIDIA para RViz y una opción explícita `show_robot_overlay`.

Se encuentra `thesis_perception/src/visualization_overlay_node.cpp`, añadido al CMake actual, que transforma la nube filtrada a `j2n6s300_link_base` para visualizarla. La matriz está en `thesis_perception/config/d435i_overlay_experimental.yaml`; el nodo no publica esa matriz como TF. El README y la configuración la identifican como ajuste específico del montaje y no como calibración. El RViz experimental asociado está en `thesis_perception/rviz/d435i_jaco_experimental.rviz`.

La opción de superposición requiere un `robot_state_publisher` y estados articulares existentes; no inicia por sí misma un adaptador ni ordena movimiento. La transformación debe revisarse si cambia el montaje.

### 3. Separación visual de brazo y resto

El prototipo `/tmp/codex_arm_separation.py` muestrea las mallas visuales del robot, transforma cada enlace con TF en el instante de la nube y clasifica como brazo candidato los puntos a menos de 20 mm de los puntos de malla muestreados. El resto se publica por separado. La comparación es aproximada: el muestreo de superficies, la geometría, la alineación y la profundidad tienen incertidumbre.

Según el contexto compartido por el usuario, una captura mostró aproximadamente **29.700 puntos de brazo** y **22.400 de resto**, usando 13 enlaces. Se informó una frecuencia cercana a **3 Hz** y descarte de capturas sin transformaciones correspondientes. No se conservaron aquí los datos crudos de aquella medición.

### 4. Margen de articulaciones e incertidumbre

El prototipo `/tmp/codex_joint_margin.py` conserva la categoría de brazo a menos de 20 mm y separa como inciertos los puntos restantes con distancia a malla menor de 60 mm y distancia a un centro articular menor de 140 mm. La nube restante contiene los otros puntos finitos. Estos umbrales son experimentales.

La conversación anterior informó unos **2.260 puntos inciertos** en una captura y una visualización naranja opcional en `/tmp/codex_segmentacion_margen.rviz`. Es una observación puntual de la que aquí no hay captura numérica original verificable.

Un prototipo posterior, `/tmp/codex_capsule_margin.py`, agrega a la incertidumbre los puntos dentro de cápsulas configuradas para segmentos del brazo. Esto **amplía la definición de incertidumbre** respecto del margen articular inicial: no debe presentarse como una clasificación idéntica. El código lee los radios y extremos de `src/thesis_core/config/jaco_capsules.yaml`.

### 5. Ruta acelerada C++/GPU ya existente

Se identificaron prototipos posteriores en `/tmp/codex_cloud_fast_seg/` y `/tmp/codex_early_test/`, y el clasificador OpenGL de `/tmp/codex_gpu_mask_early.cpp`. La versión activa observada usaba `/tmp/codex_early_test/build/codex_fast_seg` y un proceso GPU; se configuró para publicar en:

- `/codex/segmentacion/brazo_candidato`;
- `/codex/segmentacion/margen_incierto`;
- `/codex/segmentacion/resto_candidato`.

El C++ recibe la nube de cámara directamente, combina el filtro radial de 0,10 a 1,50 m y la transformación experimental, mantiene muestras recientes, consulta TF a la marca de tiempo de la nube y envía puntos, mallas y cápsulas al clasificador GPU. El shader usa umbrales de 20 mm y 60 mm, y añade cápsulas a la clase incierta. La ruta fusionada evita pasar obligatoriamente por la nube de superposición anterior. No publica una transformación calibrada y sigue siendo una salida de visualización.

La prueba sintética `/tmp/codex_gpu_compare_early.py` comparó **8.000 puntos**, incluidos puntos próximos a mallas, contra una referencia CPU con cápsulas: **0 diferencias** en esa muestra. El ejecutable informó `NVIDIA GeForce RTX 3050 Ti Laptop GPU`. Esto verifica el caso sintético ensayado, no la exactitud de la extrínseca ni la equivalencia completa para todas las nubes reales. No se dispone aquí de una comparación documentada de las tres clases con la misma captura real entre las versiones antigua y activa.

### 6. Mediciones observadas en la continuación

Las siguientes mediciones se hicieron por separado con sondas temporales de solo lectura; los intervalos no son simultáneos y el propio cliente puede afectar la recepción de nubes grandes:

| Magnitud | Resultado | Alcance |
|---|---:|---|
| Resto segmentado | 158 muestras; 22,52 Hz recibidos; mediana de 20.118 puntos | Ventana nominal de 7 s; intervalo máximo entre recepciones 352 ms. |
| Antigüedad del resto | 136 muestras; mediana 64 ms; máximo 80 ms | Reloj del nodo menos marca de la nube; depende de sincronización de relojes. |
| Nube de cámara | 160 muestras; 25,39 Hz recibidos; mediana de 183.498 puntos | Ventana nominal de 7 s; intervalo máximo entre recepciones 103 ms. |
| Intervalos entre marcas de tiempo | Aproximadamente 29,98 Hz para ambas sondas | Derivado de la mediana de diferencias entre marcas; no significa que se reciban todas las capturas. |

Los 22,52 Hz no son una medición controlada frente a los 3 Hz históricos: cambian la implementación, la definición de incertidumbre y pueden cambiar condiciones, carga y método. No se ha demostrado una mejora equivalente en cada etapa por separado ni funcionamiento sostenido a 30 Hz.

### 7. Alternativa Python ensayada, no instalada ni activada

Se preparó `/tmp/codex_joint_margin_fast.py` a partir del margen articular original. Paraleliza la búsqueda de distancias a malla con cuatro hilos por defecto (`CODEX_SEGMENT_WORKERS` permite ajustar el número), consulta la distancia articular solo para candidatos entre 20 y 60 mm, y selecciona muestras según disponibilidad de TF en lugar de imponer los 150 ms fijos del prototipo anterior. Cada transformación de enlace sigue solicitándose a la marca de la captura; si falla, no publica esa captura.

La prueba `/tmp/codex_validate_joint_fast.py` comparó las tres clases de la clasificación sobre **55.014 puntos sintéticos**, incluidos valores no finitos y algunos puntos límite: **0 diferencias por clase**. En 10 repeticiones, la mediana de la etapa de clasificación fue **80,5 ms** en la referencia de un hilo y **18,7 ms** en la alternativa de cuatro hilos. No incluye adquisición, construcción de la nube, TF, publicación ni latencia total. La prueba usa mallas de la sesión con puntos sintéticos y no acredita equivalencia completa en datos reales. Además, esta alternativa corresponde al margen articular original **sin el añadido de cápsulas**; no es un reemplazo equivalente de la ruta GPU activa.

Se ensayó también dividir las búsquedas por proximidad articular antes de calcular la distancia a malla; en una prueba inicial fue más lento (69,13 frente a 64,87 ms) y se descartó. La alternativa Python no se inició ni sustituyó al proceso activo; no se ha demostrado que mejore la cadena C++/GPU en vivo.

## Correcciones del registro de conversación

Una inspección inicial dentro de un entorno aislado no mostró procesos y llevó a comunicar que la cámara y RViz no estaban activos. Esa afirmación era incorrecta para el sistema anfitrión. Una consulta ROS posterior fuera de ese aislamiento identificó `/camera/d435i` y `/codex_segmentacion_cpp`; una revisión de procesos confirmó además el lanzamiento de percepción, RViz de segmentación y el ejecutable GPU. Las mediciones de la sección 6 se realizaron después de esa corrección.

Durante la revisión se observó un lanzamiento físico preexistente con salida de hardware habilitada en sus argumentos. La presente auditoría y las sondas descritas no ordenaron movimiento ni alteraron el proceso; el estado observado de un argumento no demuestra por sí mismo armado, movimiento o condiciones del área.

## Límites y pendientes

1. La matriz visual no es una extrínseca calibrada y no se ha validado frente a posiciones conocidas para la sesión actual.
2. El brazo candidato, el margen y el resto pueden contener errores; un objeto próximo puede clasificarse como brazo o incierto, y una zona ocluida no es espacio libre.
3. La ruta activa tiene dependencias y rutas absolutas en `/tmp`, datos de malla muestreados y parámetros específicos de sesión. No es una implementación persistente ni reproducible tras limpiar archivos temporales.
4. No se ha medido una comparación completa y simultánea de entrada/salida, pérdida de muestras, latencia por etapa y equivalencia de las tres clases con capturas reales idénticas, ni estabilidad prolongada bajo carga.
5. El resultado no está validado para conectar la salida al supervisor ni para decisiones de seguridad. La ruta de percepción general mantiene su candidato separado de `/thesis/obstacle_input` según el README.
6. No se realizaron compilaciones ni pruebas nuevas del proyecto al redactar este documento. Las pruebas y cifras consignadas arriba son las observadas antes de redactarlo; los cambios preexistentes ajenos a esta cadena no se auditan en detalle aquí.

## Identificación de artefactos inspeccionados

Hashes SHA-256 al redactar el informe; los de `/tmp` solo permiten detectar cambios mientras los archivos sigan existiendo:

| Archivo | SHA-256 |
|---|---|
| `/tmp/codex_arm_separation.py` | `82b670c2b3d1b03527a6da245ddeab55f2a283f10432fdb76f8111e1b4c3048c` |
| `/tmp/codex_joint_margin.py` | `9f4b996c63ba032c4ce8405499f6f80f23a784ed92197f83315001312a7fa000` |
| `/tmp/codex_joint_margin_fast.py` | `2dcd76a97e3901946a881067fefd44cb4e0aad1d5b524e7a2a38a5f5671b529d` |
| `/tmp/codex_early_test/early.cpp` | `d99f098a987dd41d62553e91e90772d3a83e17919f096bf541acd94d75873e7f` |
| `/tmp/codex_gpu_mask_early.cpp` | `73af88a4e292ce5d66993f8801a02e5126b28e35590ea3821f36e5e264f4f0b6` |
| `/tmp/codex_mesh_points.bin` | `5570c05fde2c1974fa4d05f63b15c8ea75bc4293c10cee5ef4f8b3ecc23cb98f` |
| `thesis_perception/config/d435i_overlay_experimental.yaml` | `e6fb608c0730c1defb8ece5886222fe0eccc364390a24f32d0856bb718ec0574` |
| `thesis_perception/src/visualization_overlay_node.cpp` | `70cc718d3e744c1e6ef53bf33578606fba0ded6e285e61d3284132b903b8a0f1` |

Los informes anteriores de las fases 8P a 8T se encuentran en este mismo directorio y permiten seguir el trabajo previo de máscaras, alineación y extrínseca candidata. Su contenido corresponde a sus propias fechas y sesiones.
