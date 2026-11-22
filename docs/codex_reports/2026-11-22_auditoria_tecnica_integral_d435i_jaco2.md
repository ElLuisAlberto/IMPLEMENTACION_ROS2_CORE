# Auditoría técnica integral de la sesión D435i–JACO2

Fecha: 22 de noviembre de 2026. Zona de lectura: Asia/Manila (UTC+08:00). Documento elaborado a partir de observaciones ROS de solo lectura, archivos del proyecto, prototipos experimentales y un rosbag ya grabado. No se han enviado órdenes de movimiento ni se ha cambiado la configuración activa para elaborar esta auditoría.

## 1. Alcance y naturaleza de la evidencia

Se documentan la configuración observada, la postura publicada por el robot, la cámara, los marcos, la superposición experimental, el uso de ChArUco, la segmentación y las mediciones de velocidad y residuos realizadas en la sesión. No se dispone de todos los datos históricos que podrían exigirse para reproducibilidad metrológica: las lecturas actuales se hicieron secuencialmente, varias mediciones anteriores solo constan en la conversación y no se ha recuperado una calibración extrínseca físicamente validada. Las copias de los prototipos no convierten el experimento en un producto terminado.

- Evidencia de la auditoría: [`evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/`](../../evidence/codex_reports/2026-11-22_auditoria_tecnica_d435i_jaco2/).
- Histórico previo de esta sesión: [auditoría de segmentación y velocidad](2026-11-22_auditoria_historico_segmentacion_d435i_jaco2.md).
- Rosbag de la sesión: [`2026-11-22_segmentacion_brazo_10s/metadata.yaml`](../../evidence/codex_reports/2026-11-22_segmentacion_brazo_10s/metadata.yaml); contiene imágenes de un laboratorio y debe tratarse según la política de datos correspondiente.
- Referencia Git: `cce819183c3d49add101e88ad395ca5f418c4ad2`; el árbol contenía cambios y archivos nuevos sin confirmar. No atribuir todo el estado modificado a una única tarea. Se conserva `git_status.txt` y un diff parcial de percepción, no un volcado completo del proyecto.
- Cada lectura ROS fue hecha por un cliente independiente y finalizó antes de comenzar la siguiente. **No son todas del mismo instante**. Las marcas de tiempo de los mensajes y el rosbag deben usarse para emparejar datos, no la proximidad entre secciones de este informe.

## 2. Equipo, software y procesos

| Elemento | Dato observado | Procedencia y límite |
|---|---|---|
| Cámara | Intel RealSense D435i, USB ID `8086:0b3a` | `lsusb`; no equivale a identificar número de serie individual. |
| Firmware de cámara | `5.17.0.10` | `d435i_startup_check_codex.json`, captura anterior de la sesión; no fue una nueva consulta directa de firmware durante la auditoría. |
| Robot | Kinova JACO2, descripción `j2n6s300`, dispositivo USB Kinova `22cd:0000` | Nombres ROS y `lsusb`; la pose física de instalación no ha sido medida aquí. |
| GPU | NVIDIA GeForce RTX 3050 Ti Laptop, 4096 MiB, capacidad CUDA 8.6 | `nvidia-smi`; driver `595.99.02`. La segmentación usa OpenGL compute, no se afirma uso de CUDA. |
| Sistema | Ubuntu kernel `6.8.0-138-generic`, x86_64; ROS 2 Humble | `sistema.txt`, `entorno.txt`. |
| Software de percepción | `ros-humble-realsense2-camera` 4.58.4; OpenCV 4.5.4; Python 3.10.12; NumPy 1.21.5; SciPy 1.8.0 | `versiones_paquetes.txt`, `entorno.txt`; la consulta de paquete con nombre exacto `librealsense2` no encontró paquete, por lo que su versión no queda establecida. |

Nodos observados incluyen `/camera/d435i`, `/robot_state_publisher`, `/pointcloud_preprocessor`, `/visualization_overlay`, `/perception_health`, `/codex_segmentacion_cpp`, RViz y nodos de la arquitectura del robot. Inventario íntegro: `nodos.txt`, `topicos.txt`, `procesos.txt`; entradas y salidas de segmentación: `info_segmentacion.txt`. La primera consulta de información de cámara falló por descubrimiento; el reintento tuvo código 0, conservándose ambos resultados y `estado_capturas.txt`.

El proceso de sistema físico ya estaba activo, con argumento `hardware_output_enabled:=true` y GUI habilitada. Esto **no demuestra** que estuviera armado o en movimiento, ni que el área estuviera despejada. Este informe no habilita pruebas físicas. No se enviaron órdenes desde las sondas empleadas.

## 3. Configuración efectiva de la D435i

El volcado completo de parámetros está en `parametros_camara.txt` y refleja lo que publicó el nodo al consultarlo. Resumen:

| Parámetro | Valor |
|---|---|
| Profundidad | 848 × 480, 30 Hz nominal, formato Z16 |
| Color | 1280 × 720, 30 Hz nominal, RGB8 |
| Alineación | `align_depth.enable=true`, sincronización habilitada |
| Ajuste de profundidad | `visual_preset=4` (High Density según README), emisor habilitado, potencia 150 |
| Exposición de profundidad | Automática desactivada; exposición 10.000 µs; ganancia 16 |
| Color | Exposición y balance de blancos automáticos habilitados |
| Nube | Habilitada, no organizada, textura color, QoS de sensor |
| Filtros | Decimación, espacial, temporal, relleno y voxel desactivados en la configuración observada |
| IMU | Acelerómetro y giróscopo desactivados |
| Rango de la cadena visual | 0,10–1,50 m radial respecto a cámara; puntos fuera se descartan |

El parámetro de selección `serial_no` está vacío; por tanto, **el número de serie individual de la cámara no quedó identificado**. `tf_static` y la extrínseca interna de cámara se pueden consultar en el bag; una lectura aislada de `tf_static` no garantiza capturar todos sus publicadores.

### Intrínsecos publicados por CameraInfo

Las matrices se citan como valores publicados, no como nueva calibración independiente. Las dos lecturas son de instantes diferentes. Distorsión publicada `plumb_bob`, cinco coeficientes cero en ambos mensajes.

Color, frame `d435i_color_optical_frame`, 1280 × 720 píxeles:

```text
K_color = [ 908.155823    0.000000  644.467651
              0.000000  908.253906  377.152924
              0.000000    0.000000    1.000000 ]
```

Profundidad, frame `d435i_depth_optical_frame`, 848 × 480 píxeles:

```text
K_depth = [ 430.521637    0.000000  423.030945
              0.000000  430.521637  233.705597
              0.000000    0.000000    1.000000 ]
```

Valores completos, matrices de proyección, dimensiones y marcas: `camara_color_info.txt` y `camara_depth_info.txt`. Para imágenes de profundidad alineadas al color hay que usar su `CameraInfo` correspondiente; no extrapolar estas matrices a otro flujo.

## 4. Postura del brazo y TF

`joint_states.txt` registra una muestra con marca Unix `1795318782.173634552` (22-11-2026, 11:39:42.173635 UTC+08:00), frame `j2n6s300_link_base`. Son valores **publicados**, sin convertirlos en medición metrológica independiente; no son una orden de movimiento.

| Articulación | Posición publicada (rad) | Conversión directa (°) |
|---|---:|---:|
| `j2n6s300_joint_1` | 2,933452 | 168,074 |
| `j2n6s300_joint_2` | 2,567805 | 147,124 |
| `j2n6s300_joint_3` | 0,983226 | 56,335 |
| `j2n6s300_joint_4` | 5,608686 | 321,354 |
| `j2n6s300_joint_5` | 0,114834 | 6,580 |
| `j2n6s300_joint_6` | 1,319417 | 75,597 |

Los seis valores publicados para dedos y puntas fueron cero; sin una validación de la fuente no debe concluirse la apertura física exacta. Se publicaron velocidades cero y `effort` vacío; ello no sustituye una medición independiente de movimiento. La conversión a grados no normaliza ángulos.

En una consulta TF **posterior**, `j2n6s300_link_base` → `j2n6s300_end_effector` se informó aproximadamente como traslación `[0,273, 0,021, 0,551] m` y cuaternión `(x,y,z,w)=[0,220, 0,706, 0,363, 0,567]`; RPY aproximado `[1,692, 0,696, 1,917] rad`. Los valores de `tf2_echo` están redondeados y son una pose calculada del modelo, no una medición externa. Ver `tf_base_efector.txt` para marcas y matriz. Las lecturas TF y de articulaciones no son simultáneas; no presentar esta pose como composición exacta de la tabla anterior. `tf_dinamico.txt`, `tf_estatico.txt` y el bag conservan otros marcos; las lecturas puntuales no equivalen a una reconstrucción temporal completa.

## 5. Superposición experimental cámara–modelo

Convención: `p_base = R * p_depth + t`, con metros y los puntos en `d435i_depth_optical_frame`. El ajuste visual guardado y usado por el prototipo fusionado es:

```text
T_base_depth =
[  0.9967828338  -0.0306131376   0.0740730590  -0.0499898445 ]
[ -0.0638462577   0.2554137241   0.9647214546  -0.6588873102 ]
[ -0.0484524265  -0.9663470729   0.2526374814   0.2456638098 ]
[  0              0              0              1            ]
```

Archivo: `configuracion/d435i_overlay_experimental.yaml`; coincide con el resultado `prototipos/codex_fit_holdout_result.npz` y con la matriz incorporada en `prototipos/early.cpp`. El `visualization_overlay_node` transforma puntos para visualización y **no publica esta matriz como TF**. Esta relación es un ajuste de escena, no una extrínseca calibrada ni una pose de instalación garantizada; se invalida para fines de reproducción si cambian el montaje, la cámara o la base.

### Procedimiento reconstruido desde los prototipos

1. Se obtuvo una nube filtrada, la descripción URDF y transformaciones de enlaces; las capturas y matrices disponibles se preservaron como `codex_live_fit.npz`, `codex_new_fit.npz`, `codex_fit_training_capture.npz` y `codex_live_robot.urdf`.
2. Los scripts muestrean superficies de mallas visuales, forman una nube del modelo y seleccionan candidatos oscuros de la nube (`media RGB < 85`) como aproximación del brazo. No es una máscara semántica verificada; sombras y objetos negros pueden contaminarla.
3. El ajuste inicial usa vecinos más próximos y mínimos cuadrados robustos con varias inicializaciones de giro; después hay ajuste con subconjunto de entrenamiento y evaluación retenida. Los scripts utilizan muestreos aleatorios con semillas indicadas en el código y seleccionan puntos en función de candidatos previos.
4. El tablero se excluye de la selección del brazo mediante una región expresada en su marco estimado; más detalles en la sección 6. Se generó un candidato refinado, pero la matriz guardada en la configuración corresponde al candidato `holdout`, no al refinado.
5. La captura `codex_new_snapshot.py` usa TF **más reciente** al obtener datos del ajuste, no garantiza TF a la marca exacta de la nube. Tampoco hay evidencia suficiente para tratar los subconjuntos o sus métricas como validación metrológica independiente. El filtrado de segmentación posterior sí solicita TF de los enlaces a la marca de la nube.

Los scripts originales y NPZ se preservan en `prototipos/`; rutas absolutas internas, dependencias, aleatoriedad, mallas instaladas y contexto de ROS deben revisarse antes de ejecutar. Algunas métricas se imprimieron en terminal y no se conservó su salida completa: no se inventan aquí valores de precisión o error.

## 6. Función de ChArUco

El tablero preparado en [fase 8U](2026-10-22_fase8u_preparacion_charuco_temporal.md) usa diccionario `DICT_4X4_50`, 7 × 5 cuadros, cuadrado nominal de 30 mm y marcador de 22 mm; patrón nominal 210 × 150 mm, 24 esquinas interiores. El PDF, PNG, especificación, instrucciones y plantilla de pose se encuentran en [`evidence/codex_reports/2026-10-22_fase8u_charuco/`](../../evidence/codex_reports/2026-10-22_fase8u_charuco/). Las medidas nominales requieren verificación física tras impresión.

La utilidad de ChArUco es detectar correspondencias conocidas entre esquinas del patrón y píxeles, y estimar mediante PnP la pose del tablero respecto a la cámara con los intrínsecos correspondientes. Si además se conoce de forma medida y trazable la pose del tablero respecto al marco del robot, es posible componer una candidata de extrínseca cámara–robot y evaluar reproyección. Un tablero visible **por sí solo** no fija la cámara respecto al robot.

En los scripts de esta sesión el archivo `codex_board_camera.npy`, junto con la transformación profundidad–color de la captura, permite construir una región alrededor del tablero y excluirla al ajustar los puntos oscuros del brazo (`codex_fit_holdout.py`). Esa utilización evita que los cuadros negros contaminen el ajuste; **no demuestra** que la matriz de superposición guardada sea una calibración basada en una pose de tablero medida. No se ha reconstruido aquí toda la procedencia de esa matriz de tablero ni sus incertidumbres. El informe previo 8U describe preparación, no una calibración física completada.

El código modificado `charuco_observational_once.py` incorpora una conversión entre el marco legacy de OpenCV (origen inferior izquierdo) y el contrato documentado (origen superior izquierdo, x derecha, y abajo y z hacia el papel); hay pruebas sintéticas modificadas. Este informe no vuelve a ejecutarlas ni acredita validación física.

## 7. Nube y separación visual

Flujo experimental activo observado:

```text
D435i /camera/d435i/depth/color/points
  → filtro radial 0,10–1,50 m + T_base_depth experimental
  → consulta TF de eslabones en la marca de captura
  → distancias a mallas muestreadas y cápsulas en GPU
  → /codex/segmentacion/brazo_candidato
    /codex/segmentacion/margen_incierto
    /codex/segmentacion/resto_candidato → RViz
```

La cadena general de percepción también recibe la nube y publica `/thesis/perception/points_filtered`; su superposición opcional publica `/thesis/visualization/points_experimental`. En la ruta fusionada activa se duplica parte de este procesamiento. El publicador de nube de cámara y las salidas inspeccionadas usan `BEST_EFFORT`, durabilidad `VOLATILE`; la aplicación usa cola reciente de tamaño 1 para las nubes. Los datos detallados de endpoints están en `qos_camara.txt`, `qos_segmentacion.txt`.

Se usaron **13 enlaces con mallas muestreadas** en el prototipo. Los puntos a menos de 20 mm de la malla muestreada se etiquetan como brazo candidato. Entre los demás, se etiquetan como inciertos los que están a menos de 60 mm de la malla y a menos de 140 mm de un centro articular, o dentro de alguna de las seis cápsulas. El resto de puntos válidos forma el resto candidato. Las desigualdades estrictas y el cálculo concreto se encuentran en `codex_gpu_mask_early.cpp`.

| Cápsula | Desde → hasta | Radio (m) |
|---|---|---:|
| `base_to_shoulder` | base → link 1 | 0,105 |
| `shoulder_to_upper_arm` | link 1 → link 2 | 0,105 |
| `upper_arm_to_forearm` | link 2 → link 3 | 0,085 |
| `forearm_to_wrist_1` | link 3 → link 4 | 0,075 |
| `wrist_1_to_wrist_2` | link 4 → link 5 | 0,085 |
| `wrist_2_to_tool` | link 5 → efector | 0,090 |

Son radios de envolventes del archivo de configuración, no mediciones individualmente validadas de superficie. El margen de incertidumbre puede incluir objetos reales; el resto puede contener residuos del brazo; lo ocluido no se convierte en libre. El muestreo y la distancia aproximada no sustituyen una prueba de colisión exacta. Salidas aisladas de la supervisión preventiva; no validadas para decisiones de seguridad.

La pantalla RViz usa `/tmp/codex_segmentacion_margen.rviz`, preservada en `prototipos/`. El indicador de fps de RViz mide dibujo de la ventana y no frecuencia de nubes. La ruta fusionada depende de ficheros de `/tmp`, matriz y archivos concretos; se preservaron fuentes, binarios y mallas muestreadas por trazabilidad, pero no son una instalación portable ni una interfaz estable. La configuración copiada corresponde al estado en el momento de auditar, no prueba por sí sola los parámetros de un proceso que ya haya cargado otra versión.

## 8. Rosbag, residuos y frecuencia

Rosbag SQLite3 de **10,666 s**, 1,7 GiB, 1.961 mensajes; no se trató de un ensayo de movimiento. Incluye nube original (231 mensajes), RGB (280), CameraInfo color (289), brazo (225), margen (253), resto (254), estados (212), TF (214), TF estática (2) y robot_description (1). Las capturas de un tópico pueden no coincidir con las de otro por inicio de suscripción y pérdidas; no se debe inferir equivalencia o pérdida de un nodo solo por contar mensajes. La nube resto cubrió 9,939 s; la nube original, 9,539 s. Información exacta en `bag_info.txt` y `metadata.yaml`.

Se analizaron 254 nubes de resto con distancia a las mallas muestreadas transformadas por TF del bag. En 214 el resto estaba vacío; 40 tenían en conjunto 477 puntos y un máximo de 37 por nube. Ninguno de esos puntos quedó a menos de 100 mm de la malla muestreada; de los grupos de al menos tres puntos (vecindad 12 mm), el más cercano quedó a aproximadamente 578 mm. Resultados y código en [`2026-11-22_segmentacion_analisis_10s/`](../../evidence/codex_reports/2026-11-22_segmentacion_analisis_10s/). El RGB de esta grabación no muestra el tablero anterior. Esta escena no demuestra comportamiento cerca de un objeto y no justificó ampliar el margen.

Mediciones de observación en vivo, ventanas separadas (no simultáneas), derivadas de sondas conservadas en `prototipos/`:

| Señal | Ventana | Frecuencia media recibida | Intervalo mediano | Intervalo máximo |
|---|---:|---:|---:|---:|
| RGB | 15 s | 27,71 Hz | 33,5 ms | 136,1 ms |
| Nube de cámara | 20 s | 24,78 Hz | 33,9 ms | 276,2 ms |
| Resto segmentado | 20 s | 22,73 Hz | 31,8 ms | 350,5 ms |
| TF | 15 s | 20,00 Hz | 50,0 ms | 52,3 ms |

Edad de recepción del resto: mediana 63,7 ms y percentil 95 de 69,3 ms, calculada como reloj del cliente menos sello del mensaje. Estas cifras incluyen posibles efectos del cliente, DDS, CPU y reloj y no son una caracterización estadística de larga duración. El diagnóstico posterior publicó 25 Hz de nube filtrada en ese instante. TF a 20 Hz no implica necesariamente salida máxima de 20 Hz: el código puede consultar transformaciones interpoladas en la marca de captura.

La comparación sintética GPU/CPU conservada usó 8.000 puntos y tuvo 0 diferencias para ese caso, con definición de margen que incluye cápsulas. Una variante Python **no activa**, sin cápsulas, se comparó con su referencia sobre 55.014 puntos sintéticos: 0 diferencias en tres clases y medianas de clasificación de 80,5 y 18,7 ms (uno y cuatro hilos); no incluye toda la cadena y no demuestra mejora de la ruta GPU activa. Los detalles y advertencias constan en el [histórico](2026-11-22_auditoria_historico_segmentacion_d435i_jaco2.md).

## 9. Riesgos de interpretación y datos necesarios para tesis

- No presentar el ajuste visual como calibración, ni los umbrales o cápsulas como límites de seguridad validados.
- El experimento utiliza una postura observada y una escena concreta; no prueba robustez ante otras posturas, objetos cercanos, movimiento, oclusión ni cambios de iluminación.
- Las capturas ROS de la auditoría son secuenciales. Para una evaluación geométrica rigurosa, usar muestras sincronizadas, conservar sellos y TF, medir la pose del tablero respecto de la base y reportar incertidumbre, residuos, muestras excluidas y condiciones físicas.
- La evaluación de velocidad necesita repeticiones más largas y comparables, con mediciones simultáneas y carga controlada; las sondas pueden afectar el sistema. La configuración nominal de la cámara es 30 Hz, no el rendimiento garantizado.
- No se conoce el número de serie específico ni se preservó una medición física completa de la posición de cámara, base y tablero en esta auditoría. Las salidas antiguas de terminal que no se guardaron no pueden recuperarse con certeza.
- Para uso en tesis, conservar también la versión exacta del código y dependencias, el bag y su contexto, los datos experimentales y la política de tratamiento de imágenes; no citar las observaciones de esta sesión como validación general.

## 10. Integridad y archivos

El directorio de evidencia contiene parámetros, inventarios, TF, postura, QoS, diagnósticos, fuentes temporales, datos de ajuste, binarios, configuración y `SHA256SUMS.txt`. El rosbag y el análisis permanecen en sus directorios independientes. Las rutas originales están documentadas en el histórico; las copias permiten preservar lo que estaba en `/tmp`, pero varios scripts mantienen rutas de esa ubicación y no se han adaptado ni probado como paquete portable. La suma SHA-256 identifica bytes archivados; no prueba autoría ni exactitud de los datos.
