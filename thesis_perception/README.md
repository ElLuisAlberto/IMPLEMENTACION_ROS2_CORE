# thesis_perception

Robot-independent RGB-D processing for the preventive thesis architecture.
The package can be characterized with the Intel RealSense D435i before any
simulated or physical manipulator is connected.

## Guía rápida de archivos

Para abrir la configuración de RAFT usada actualmente, sigue los comandos de
[AGENTS.md](../AGENTS.md#configuración-de-percepción-que-el-usuario-quiere-recuperar).
La configuración utiliza la cámara, RAFT experimental, el barrido de líneas y
RViz; no inicia control del robot.

| Archivo | Función |
| --- | --- |
| `scripts/point_flow_raft_preview_experimental.py` | Predictor RAFT experimental actual y filtros de movimiento. |
| `scripts/point_flow_preview.py` | Código compartido de recepción RGB-D, necesario también para el predictor actual. |
| `src/raft_lines_node.cpp` | Generación del barrido de líneas para la vista actual. |
| `rviz/raft_adaptive_preview.rviz` | Vista actual de RGB, predicción y barrido. |
| `scripts/alternativas/point_flow_raft_preview.py` | Predictor RAFT anterior, conservado para comparación. |
| `src/alternativas/raft_sweep_node.cpp` | Barrido anterior de cubos para el predictor RAFT anterior. |
| `src/alternativas/cloud_prediction_node.cpp` | Predictor alternativo basado en grupos de puntos. |

La organización de los scripts y la distinción entre código activo, compartido
y de comparación está resumida en
[`scripts/README.md`](scripts/README.md). No se eliminan las alternativas:
conservan valor como referencia reproducible y sus nombres instalados se
mantienen para no romper experimentos existentes.

Las alternativas siguen compilándose y se pueden ejecutar por sus nombres
anteriores; la configuración actual no las inicia. No se deben iniciar varios
predictores sobre los mismos tópicos de visualización simultáneamente.

## Safety boundary

The default output is:

```text
/thesis/perception/obstacle_candidate
```

It is intentionally different from `/thesis/obstacle_input`. Therefore,
starting this package cannot modify a preventive decision. The candidate must
only be routed to the supervisor after validating the workspace crop, camera
extrinsic, timestamps, uncertainty and dropout response.

## Pipeline

```text
D435i PointCloud2
  -> pointcloud_preprocessor
  -> /thesis/perception/points_filtered
  -> obstacle_extractor
  -> /thesis/perception/obstacle_candidate
```

`pointcloud_preprocessor` rejects non-finite points, optionally transforms the
cloud, rejects physically implausible ranges before voxelization, applies a
configurable workspace crop and reduces the result with a voxel grid.

`obstacle_extractor` performs Euclidean clustering, selects the nearest valid
cluster, encloses it in a conservative sphere and estimates a bounded velocity.
The output uses the generic `thesis_interfaces/Obstacle` contract.

`perception_health` publishes `READY`, `DEGRADED` or `LOST` through
`/thesis/perception/diagnostics`. A fixed camera does not require IMU data by
default.

The extractor publishes `/thesis/perception/extraction_diagnostics` on every
processed cloud. Its `has_obstacle` key distinguishes a healthy empty scene
from a detected candidate; loss of clouds is reported separately by
`perception_health`.

## Standalone mode

After building the workspace:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=true
```

RViz requests NVIDIA PRIME render offload by default. Use
`rviz_use_nvidia:=false` on hosts without a working NVIDIA OpenGL setup. This
only selects the GPU for RViz rendering; camera acquisition, preprocessing
and cloud transformation run on the CPU.

To reproduce the experimental robot/cloud overlay from the current session,
with an already-running robot state publisher and physical or simulated
`/joint_states`, use:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=true show_robot_overlay:=true start_extractor:=false
```

This does not start a robot adapter or issue motion commands. The saved
visualization-only fit is specific to the observed mounting, is not a camera
calibration, publishes no TF and is not connected to the supervisor. Recheck
it after the camera, robot base or mounting moves. It remains disabled unless
`show_robot_overlay:=true` is explicitly requested.

The standalone launch selects the D435i High Density preset, infrared
emitter power 150, manual depth exposure of 10,000 microseconds, depth at
640x480x60 and RGB at 640x480x60. It displays the RGB image and the cloud
filtered to a radial range of 0.10-1.20 m in the camera frame. Decimation,
spatial and temporal depth filters are disabled by default; voxel downsampling
is enabled at 0.01 m by default. These are experimental observation settings selected for the
current scene; they are not a validated safety configuration. The camera
profiles can be overridden with `depth_profile` and `color_profile`. The
0.10-1.20 m cutoff is radial distance from the origin of the processing frame;
in standalone mode that is the camera optical frame. The sensor may not
produce reliable depth as close as 0.10 m. RViz uses the NVIDIA GPU when
PRIME is available; camera acquisition, filtering and clustering use CPU.

## Configuración reproducible experimental de RAFT

Esta configuración recupera la sesión activa de percepción del 23 de noviembre
de 2026. Es únicamente experimental: no inicia control del JACO2 ni debe
usarse como límite de seguridad.

Distribución de ejecución:

| Componente | Procesador | Implementación | Resultado |
| --- | --- | --- | --- |
| Cámara D435i | CPU | `realsense2_camera_node` | RGB, profundidad alineada y cámara nativa |
| Comparación de distancia | CPU / OpenCV | `distance_comparison_views.py` | nubes XYZRGB filtrada y suavizada |
| RAFT experimental | GPU NVIDIA / CUDA | `point_flow_raft_preview_experimental.py` con `build/raft_env` | predicción de movimiento |
| Barrido de líneas | CPU / C++ | `raft_lines_node` | volumen visual cian |
| RViz | GPU NVIDIA mediante PRIME | `rviz2` | visualización final |

Desde la raíz del workspace, comprueba primero los procesos con
`ros2 node list --no-daemon --spin-time 2` y no dupliques nodos ya activos.
En cada terminal carga el entorno:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Inicia la cámara y la vista de D435i sin lanzar procesamiento adicional:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=false start_processing:=false start_extractor:=false \
  start_cloud_prediction:=false start_distance_comparison:=true
```

`start_distance_comparison:=true` inicia
`/comparacion_distancia_suavizado`. Ese nodo publica
`/thesis/comparacion/distancia` y `/thesis/comparacion/suavizado` como nubes
`PointCloud2` con campos `x`, `y`, `z` y `rgb`. Si no se necesita esta
comparación, conserva el valor predeterminado `false`.

Inicia el generador C++ de líneas y el predictor RAFT CUDA experimental en
terminales separadas:

```bash
ros2 run thesis_perception raft_lines_node --ros-args \
  -r /thesis/perception/raft_swept_surface_experimental:=/thesis/perception/swept_volume
```

```bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python \
  install/thesis_perception/lib/thesis_perception/point_flow_raft_preview_experimental.py \
  --ros-args -p profile_timing:=true -p sweep_mode:=cpp -p async_pairing:=true \
  -p max_valid_range_m:=1.2 -p temporal_motion_confirm:=true \
  -p adaptive_small_motion:=true -p temporal_search_3x3:=true \
  -r /thesis/perception/point_flow_raft_preview_experimental:=/thesis/perception/predicted_points
```

Abre la configuración guardada con la RTX solicitada:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/raft_adaptive_preview.rviz
```

La vista muestra RGB, predicción RAFT, superficie/líneas experimentales y las
nubes `/thesis/comparacion/distancia` y `/thesis/comparacion/suavizado` cuando
los nodos correspondientes estén activos. Para repetir la sesión completa se
deben conservar estos parámetros: 12 iteraciones, alcance radial máximo de
1,2 m, confirmación temporal en dos imágenes, búsqueda 3×3, soporte adaptativo
de 15 puntos para grupos grandes y 5 para grupos pequeños, y emparejado
asíncrono (`async_pairing:=true`). No añadas el predictor legacy,
`/pointcloud_preprocessor`, `/perception_health` ni `/cloud_prediction`.

While that launch is running, a terminal-only report can be generated with:

```bash
bash tools/realsense_d435i_test/scripts/validate_perception_standalone.sh
```

The report uses the lightweight `perception_health` counter for cloud rate.
Direct `ros2 topic hz` probes are disabled because copying large PointCloud2
messages can reduce the rate being measured. They can be enabled explicitly
for troubleshooting with `PERCEPTION_RUN_RATE_PROBES=true`.

This preserves the camera frame and does not publish a fabricated extrinsic.
The crop is disabled until the actual workspace bounds have been measured.
Accelerometer and gyroscope acquisition is also disabled by default because a
fixed RGB-D camera does not need inertial data for obstacle geometry. Enable it
explicitly with `enable_imu:=true` only after validating host IIO permissions.
The standalone launch also exposes `enable_crop`, `min_x`, `max_x`, `min_y`,
`max_y`, `min_z`, `max_z`, `enable_voxel` and `voxel_leaf_size` so the final
workspace can be tuned from the terminal without modifying source files.
Spatial and temporal filters and decimation can be enabled explicitly for
experiments. The camera PointCloud2 publisher uses the `SENSOR_DATA` QoS
profile, while downstream consumers retain only the newest sample so stale
geometry is not accumulated.

## Session segmentation and recorded evidence

The arm-removal prototype from the November 22 session is now preserved in
[`tools/perception/experimental_arm_segmentation/`](../tools/perception/experimental_arm_segmentation/README.md)
with its source, fixed mesh samples, build and startup instructions. The
standalone camera and overlay launch above **does not start this filter**.
The prior session process can still be running from `/tmp`; stop only the
relevant old visualization process before starting another publisher on the
same topics. The filter requires a compatible OpenGL/X11 environment and uses
a session-specific visual transform, not an extrinsic calibration.

The [technical audit](../docs/codex_reports/2026-11-22_auditoria_tecnica_integral_d435i_jaco2.md)
records the camera, transforms, robot state and experimental settings. The
[static test](../evidence/codex_reports/2026-11-22_prueba_filtro_estatico_resultados/RESULTADOS.md),
[initial motion test](../evidence/codex_reports/2026-11-22_prueba_filtro_movimiento_resultados/RESULTADOS.md)
and [operator-motion recording](../evidence/codex_reports/2026-11-22_movimiento_operador_resultados/RESULTADOS.md)
describe the available results and their limits. Their bags are in the
corresponding evidence directories. The repository ignores `*.db3`, so a Git
commit or clone alone will not preserve the recordings; keep a separate backup
of bag directories and their metadata. Other session changes and new files may
also remain uncommitted: check `git status` before relying on version control.
The filter and visual alignment remain experimental and are not validated for
supervisor or physical safety decisions.

## Extrinsic calibration

The processing launch can publish a static `world -> d435i_link` transform,
but the option is disabled by default. Do not enable it with zero values. Once
the measured translation and roll-pitch-yaw are available, they can be supplied
as launch arguments:

```text
publish_static_extrinsic:=true
extrinsic_parent_frame:=world
camera_link_frame:=d435i_link
extrinsic_x:=...
extrinsic_y:=...
extrinsic_z:=...
extrinsic_roll:=...
extrinsic_pitch:=...
extrinsic_yaw:=...
target_frame:=world
```

The RealSense driver publishes the internal transforms from `d435i_link` to
its optical frames. The thesis package only owns the measured transform between
the external reference and the camera body.

## Clock policy

- Standalone or physical operation: `use_sim_time:=false` and
  `restamp_output:=false`.
- Real camera with a simulated robot: `use_sim_time:=true` and
  `restamp_output:=true` at the perception gateway.

The health node detects stale timestamps and timestamps too far in the future.

## Integration gate

Before routing the candidate to `/thesis/obstacle_input`, validate:

1. Stable cloud frequency and diagnostic state `READY`.
2. Correct `world -> d435i_link` calibration.
3. Crop bounds that exclude floor, walls and irrelevant areas.
4. Candidate position against objects placed at known coordinates.
5. Empty-scene behavior and USB disconnection behavior.
6. Recorded processing latency and calibrated uncertainty.
7. Self-filter behavior when the JACO2 is later introduced.

The current extractor intentionally models one principal obstacle. Multiple
simultaneous obstacles require a future array or multi-sphere contract.

## Experimental point-cloud motion preview

This optional preview retains every finite point of the filtered cloud and
estimates independent translations for clusters tracked across frames. Points
in clusters without confirmed motion, and points too small or large to cluster,
are carried forward unchanged as a stationary hypothesis; this is not proof
that they are stationary. The tracker uses position and cluster point-count
consistency and can confuse objects when clusters split, merge, cross or become
occluded. It does not predict rotation, deformation or occluded surfaces.
It is a visualization experiment and does not publish to the supervisor.

For a fixed camera, an experimental short-lived voxel memory identifies
surfaces repeatedly observed in the same position. Clustering uses the whole
cloud; a cluster with a majority of recently stable points is not projected as
moving. When movement is confirmed for a cluster with a majority of nonstable
points, its full visible geometry is projected, including points that still
overlap the remembered area. This can reduce false movement of a background
while preserving the moving object's shape, but adjacent objects may merge. It requires approximately one second of repeated observations before a
voxel is considered stable and retains it for up to three seconds after its
last observation. It is reset on missing, invalid, reordered or stale input and
frame changes. Its parameters are `stationary_map_enabled`,
`stationary_voxel_m`, `stationary_match_m`, `stationary_warmup_s`, and
`stationary_retention_s` on `/cloud_prediction`.

A large connected cluster can contain a locally moving part, for example a
person moving an arm while their torso is still. If most of a cluster matches
the stationary memory, the remaining points are grouped into local candidates
and tracked separately. This estimates only detected visible moving portions;
it is not a human pose or full-body motion model. Small, occluded, slow or
fragmented parts may not produce a candidate, and changing visibility can
still appear as motion.

This is not a general motion segmentation or a safety guarantee. A moving
camera invalidates the fixed-scene assumption, and slow objects, newly exposed
surfaces, moving objects occupying a previously stable location, reflective
surfaces and occlusions may be misclassified. Green means an unchanged-position
hypothesis, not confirmed free space or certain rest. Scene changes require
reacquisition; a retained old surface may temporarily hide real motion.

With the D435i connected, place one rigid object approximately 0.5–1 m in front
of a fixed camera, separated from other surfaces. Start the documented launch:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=true start_cloud_prediction:=true start_extractor:=false \
  enable_voxel:=true voxel_leaf_size:=0.01
```

RViz displays the filtered cloud and the complete predicted cloud. In the
predicted cloud, green points retain their current position and orange points
belong to clusters with confirmed motion; orange points are translated by the
estimated velocity over the 0.5 s default horizon. Initially every point is
green because motion has not yet been established. Velocity is estimated over
a short window with separate thresholds for entering and exiting motion. The
velocity used for rendering is smoothed while motion remains confirmed. A
second, shorter window suppresses the older prediction after sustained braking
or reversal and requires consistent evidence before re-enabling it. This
reduces the apparent return and oscillation when stopping; it can also briefly
suppress a genuinely slow or decelerating object. The thresholds can miss slow
movement and add delay. Prediction clears on missing,
stale or invalid input. The preview uses the original observation timestamp and
camera frame; it does not provide an extrinsic calibration or compare against
the arm. Occluded surfaces are not reconstructed.

Parameters of `/cloud_prediction` include `horizon_s`, `cluster_tolerance_m`,
`min_cluster_points`, `max_cluster_points`, `association_distance_m`,
`tracking_timeout_s`, `max_speed_mps`, `velocity_window_s`,
`moving_enter_speed_mps`, `moving_exit_speed_mps`, `max_valid_range_m` and
`range_edge_margin_m`. The thresholds are experimental, not calibrated.
`/thesis/perception/prediction_diagnostics` reports active and moving track
counts, stationary reference points, unclustered points, tracks started and
lost, and the fraction of points within 2 cm of the configured maximum range. The edge warning indicates a
possible clipped object, not a measurement of the missing geometry. Change them
through a ROS parameter file or ROS arguments on a separately launched node.
The velocity estimate is sensitive to changing visible surfaces and cluster
merges; the orange cloud is a hypothesis, not a measured future or a safety
boundary. Before any eventual safety integration, evaluate latency, occlusion,
tracking identity, uncertainty, calibration and dropout behavior.

A synthetic ROS integration smoke test can be run after sourcing the workspace:

```bash
python3 thesis_perception/test/cloud_prediction_smoke.py
```

Run it only without another publisher on the preview topics: it publishes
synthetic points to the normal filtered-cloud topic and starts its own preview
node. It checks retention of all points, independent moving clusters, return to rest
and clearing after data loss. The additional `occlusion_prediction_smoke.py` checks a synthetic moving
foreground that occludes a stationary background, and
`local_motion_prediction_smoke.py` checks local motion connected to a large
stationary body;
run it in an isolated ROS domain in the same way.

## Optional point outlier filters

`pointcloud_preprocessor` can apply `outlier_method:=statistical` or
`outlier_method:=radius` after range, optional crop and voxel filtering. The
standalone and processing launches default to `outlier_method:=none`: no
particular setting has yet been validated for thin objects or safety use.
To experiment with the statistical version, for example:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=true start_cloud_prediction:=true start_extractor:=false \
  outlier_method:=statistical
```

The method parameters are `outlier_mean_k`, `outlier_stddev`,
`outlier_radius_m` and `outlier_min_neighbors`. Defaults are experimental,
not calibrated. The filtered output remains `/thesis/perception/points_filtered`.
`/thesis/perception/points_before_outlier` retains the corresponding cloud
before outlier removal, with the same timestamp and frame. It is published
when subscribed, avoiding an extra large cloud copy when unused. Both clouds
have already passed the configured range, crop and voxel steps; the topic is
not the original raw camera cloud. Outlier removal can discard real small or
sparsely sampled surfaces, so do not treat discarded points as confirmed free
space. Enabling a depth spatial filter changes the input before both outputs.

After building and sourcing the workspace, in an isolated ROS domain run:

```bash
ROS_DOMAIN_ID=84 python3 thesis_perception/test/outlier_filter_smoke.py
```

The test sends a synthetic cluster and isolated point through both methods,
checks the preserved pre-filter cloud and matching timestamps, and stops its
own processing node. Do not use a domain containing another active test with
the same topics.

## Experimental swept surface

With `start_cloud_prediction:=true`, the preview also publishes a translucent
cyan voxel marker at `/thesis/perception/swept_volume`. For every point in a
cluster whose movement is confirmed, the node samples its current position and
its constant-velocity path to the configured prediction horizon. Occupied
2 cm voxels are combined; the marker is deleted when no cluster has confirmed
motion or when input expires. It is a visualization of sampled **visible
surface trajectories**, not a reconstructed solid, uncertainty envelope or
conservative collision volume. Stationary, unassociated, occluded and outside
range points cannot be assumed free. The marker has no per-voxel time labels;
its spatial overlap with the arm alone cannot determine a collision at the
same future instant. It is not connected to the supervisor.

The standalone launch offers `sweep_enabled`, `sweep_voxel_m`,
`sweep_max_cells` and `sweep_max_steps`. Sampling aims for a maximum spacing of
half a voxel but still does not mathematically guarantee conservative voxel
coverage. `sweep_max_cells` limits marker size and `sweep_max_steps` limits
work per point. `/thesis/perception/prediction_diagnostics` reports
`sweep_cells`, `sweep_truncated` and `sweep_undersampled` and changes to a
warning if a limit makes the output incomplete. The default voxel size is a
rendering choice and is **not** a calibrated uncertainty or safety margin.
The current RViz configuration displays the marker for the normal camera view;
the robot overlay remains a separate, uncalibrated experimental visualization.

## Alternativa anterior: cámara y predicción RAFT legacy

Estos comandos corresponden a la variante anterior, no a la configuración
actual documentada en `AGENTS.md`. Se ejecutan desde la raíz del workspace,
con la D435i conectada.
La vista muestra la nube filtrada y una predicción experimental; no inicia un
robot ni evasión de obstáculos. Se necesitan tres terminales abiertas.

En la primera, inicia la cámara y el preprocesamiento:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=false start_extractor:=false start_cloud_prediction:=false
```

En la segunda, inicia la predicción RAFT:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python install/thesis_perception/lib/thesis_perception/point_flow_raft_preview.py
```

En la tercera, abre RViz con la RTX:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/point_flow_raft_preview.rviz
```

La vista RAFT incluye el marcador cian `/thesis/perception/raft_swept_surface`:
muestrea en 11 instantes, desde el presente hasta 0,5 s, los puntos visibles
con movimiento aceptado y los agrupa en vóxeles de 2,5 cm. Se borra cuando
no hay movimiento aceptado o caducan los datos. Es una visualización
experimental de superficies muestreadas: pueden quedar huecos entre muestras,
no reconstruye el interior ni las zonas ocultas y no representa incertidumbre.
La unión no conserva el instante de cada vóxel y no está conectada al control
ni constituye una envolvente de seguridad.

Hay una alternativa experimental en C++ para generar el mismo marcador. La
inferencia RAFT y la selección de puntos siguen iguales; solo se trasladan el
muestreo, la agrupación y la creación del marcador. Con la cámara ya iniciada,
en terminales separadas y con los entornos anteriores cargados:

```bash
ros2 run thesis_perception raft_sweep_node
```

```bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python install/thesis_perception/lib/thesis_perception/point_flow_raft_preview.py \
  --ros-args -p sweep_mode:=cpp
```

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/point_flow_raft_cpp_preview.rviz
```

El modo predeterminado `legacy` conserva la vista anterior. `sweep_mode:=both`
publica ambas para comparación. La nueva salida usa
`/thesis/perception/raft_swept_surface_cpp` y caduca después de 0,6 s. No se
deben iniciar dos instancias RAFT con los mismos tópicos. La frecuencia depende
de la escena y los marcadores grandes pueden perderse con transporte best effort.

### Variante experimental compilada

`point_flow_raft_preview_experimental.py` es una variante separada que usa
`torch.compile(..., mode="reduce-overhead")` con los mismos pesos, resolución
256×192, tipo Float32 y 12 iteraciones RAFT. Publica en
`/thesis/perception/point_flow_raft_preview_experimental` y no reemplaza ni
modifica el nodo `legacy`. Debe ejecutarse sin el otro nodo RAFT activo:

Esta variante también conserva las operaciones de correspondencia, velocidad,
Kalman y soporte de vecindad en CUDA. Usa muestreo nearest-neighbor por índices
para esta prueba; no sustituye aún al remapeo bilineal del nodo legacy.
El suavizado espacial usa una mediana 5×5 por componente de velocidad, como
el predictor anterior, para reducir valores aislados; puede atenuar movimiento
local pequeño y no garantiza eliminar falsas detecciones.
Un punto se muestra en movimiento cuando supera 0,06 m/s y hay al menos quince
puntos móviles (incluido él mismo) en su ventana de 5×5 píxeles. Este filtro
puede ocultar movimiento real de objetos pequeños o de bordes finos.
La opción experimental `-p temporal_motion_confirm:=true` exige que el
candidato también aparezca en el punto correspondiente de la imagen anterior.
Puede retrasar o descartar movimiento breve y no elimina ruido persistente;
por defecto está desactivada para conservar la configuración anterior.
Con `-p temporal_search_3x3:=true` la confirmación temporal conserva las
coincidencias exactas y también acepta candidatos en un entorno 3×3 con
profundidad y velocidad compatibles. Es experimental: puede recuperar puntos
en movimiento y también aumentar falsas detecciones; no sustituye la
validación con una referencia real. Desactivar el parámetro restaura la
confirmación exacta.
La prueba opcional `-p adaptive_small_motion:=true` admite desde 5 candidatos
en 5×5 cuando sus velocidades locales tienen dirección y magnitud coherentes;
estos puntos pequeños siempre requieren confirmación temporal. Las superficies
con 15 o más candidatos mantienen el umbral anterior. La mediana y el soporte
previos pueden seguir eliminando objetos pequeños y los umbrales no están
calibrados para seguridad.
El marcador experimental cian representa cada punto móvil como una línea desde
su posición observada hasta su posición estimada a 0,5 s, con velocidad constante.
No agrupa las trayectorias en vóxeles ni representa una envolvente de seguridad.

Para generar esas líneas en un proceso C++ separado, primero ejecute
`ros2 run thesis_perception raft_lines_node` y añada
`--ros-args -p sweep_mode:=cpp` al predictor experimental. El predictor envía
los extremos de las líneas en `/thesis/perception/raft_lines_input_experimental`;
el nodo C++ publica el mismo marcador experimental con una cola de un mensaje.
En el modo predeterminado `python` no se necesita el nodo C++.

```bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python \
  install/thesis_perception/lib/thesis_perception/point_flow_raft_preview_experimental.py
```

Para medir el tiempo de procesamiento se puede añadir `-p profile_timing:=true`
a los argumentos ROS del predictor. En esta variante, cada 60 cuadros imprime
las medianas y percentiles 95 en milisegundos: `flow` no está instrumentado y
siempre muestra cero; `cpu_other` incluye la inferencia y el procesamiento
previo a publicación, y `publish` incluye la generación del barrido y las
llamadas de publicación, no la entrega ni el renderizado. `total`
abarca esas etapas desde el inicio del procesamiento del par; no mide la
antigüedad de la imagen ni las esperas antes de recibir el par. La medición está
desactivada por defecto.

Para diagnosticar por separado las etapas CUDA y el emparejamiento RGB-D,
`-p profile_stages:=true` registra medianas, percentiles y contadores. Este
perfil sincroniza CUDA entre etapas y modifica el rendimiento observado; los
contadores de entrada son callbacks recibidos por el nodo, no capturas totales
de la cámara. Se recomienda desactivarlo para la operación normal.

La opción experimental `-p async_pairing:=true` empareja RGB-D en el hilo de
recepción y procesa el par más reciente en un hilo separado. La cola mantiene
un solo par pendiente y descarta los pares sustituidos. El comportamiento
anterior sigue siendo el predeterminado (`async_pairing:=false`). Compare
latencia y predicciones antes de adoptar el modo experimental.
Para una prueba con colas ROS acotadas pueden añadirse
`-p image_queue_depth:=4 -p max_pair_age_s:=0.5 -p profile_pairing:=true`.
La profundidad predeterminada sigue siendo 1; el límite de edad se comprueba
contra el reloj ROS y, en reproducción, requiere `use_sim_time` y `/clock`.
`profile_pairing` cuenta emparejamientos y descartes sin sincronizar CUDA;
la cuenta `throttled` solo se instrumenta en el modo síncrono.

Para volver al método anterior, detén RAFT y usa `ros2 run thesis_perception
point_flow_preview.py` en la segunda terminal. En la tercera, sustituye el
archivo de RViz por `point_flow_preview.rviz`. Cada terminal necesita cargar
ROS y `install/setup.bash` como se muestra arriba.

Estado de las pruebas del 23 de noviembre de 2026: Farneback fue valorado por
el operador como estable para movimientos lentos, pero perdió correspondencias
con movimientos rápidos. RAFT se cargó en la RTX 3050 Ti y se comprobó con la
D435i y datos grabados; su precisión y estabilidad frente a Farneback aún no
se han validado con una referencia real. Con ambos predictores y RViz activos
se observaron unos 13 Hz de RAFT con puntos visibles; con solo RAFT y sin
puntos cercanos se observaron unos 21 Hz. No son garantías de rendimiento.

Las grabaciones originales quedan en `evidence/rosbags/`:

- `tesis_point_flow_live_20261123`: secuencia RGB-D para diagnóstico.
- `tesis_point_flow_fast_20261123`: comparación de movimiento lento y rápido.
- `jaco_movimiento_local_20261123`: nube filtrada usada con el predictor previo.

Los archivos `.db3` están excluidos de Git y necesitan respaldo aparte; los
metadatos por sí solos no contienen las imágenes ni las nubes. Los modelos y
el entorno RAFT están bajo `build/`, también excluido de Git. Si se borra
`build/`, se deben recrear el entorno y los pesos; el código fuente y las
configuraciones están en `thesis_perception/`. Para recrear el entorno aislado
se usaron estos comandos, que descargan varios GB y no necesitan `sudo`:

```bash
/usr/bin/python3 -m venv --system-site-packages --without-pip build/raft_env
build/raft_env/bin/python -m pip install 'torch==2.5.1' 'torchvision==0.20.1' 'numpy<2' \
  --index-url https://download.pytorch.org/whl/cu124
mkdir -p build/raft_weights
curl -fL -o build/raft_weights/raft_small_C_T_V2-01064c6d.pth \
  https://download.pytorch.org/models/raft_small_C_T_V2-01064c6d.pth
sha256sum build/raft_weights/raft_small_C_T_V2-01064c6d.pth
```

El SHA-256 del archivo utilizado fue
`01064c6dba73b0fc9fc8edf772248560a00a3acfd62ac6677e9eeebad9680e27`.
Para reconstruir el paquete después de limpiar `install/`, ejecuta
`source /opt/ros/humble/setup.bash` y `colcon build --packages-select thesis_perception`
desde la raíz antes de los comandos de arranque.

## Experimental per-pixel RGB-D motion preview

`point_flow_preview.py` is a separate visualization experiment. It uses aligned
color and depth images matched within 8 ms from the fixed D435i, computes backward and forward
Farneback optical flow through OpenCV UMat with an NVIDIA OpenCL device, rejects
inconsistent correspondences and filters each matched pixel's 3D velocity with
a scalar Kalman update. A 5x5 median and minimum valid-neighbor support
reduce isolated velocity estimates but can suppress small or thin moving objects.
Orange points are projected over 0.5 s; green points
remain at their observed location. The cloud is reconstructed from aligned
depth at 256x192 and is separate from `/thesis/perception/points_filtered`.
The latter remains visible for comparison. Frame loss clears the preview.
The preview processes frames at a minimum 33 ms interval; actual rate depends
on load.
The scalar covariance, speed threshold, matching gates, and horizon are
experimental, not calibrated. Motion toward the camera, textureless surfaces,
occlusions, illumination changes and overlapping objects can be missed or
misclassified. Neither this display nor its point output is a safety input.

With the standalone camera already running, after building and sourcing:

```bash
ros2 run thesis_perception point_flow_preview.py
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/point_flow_preview.rviz
```

The node requires an NVIDIA OpenCL device and rejects startup if unavailable.
OpenCL device selection alone does not demonstrate that every OpenCV operation
ran on the GPU. It does not change or stop any existing cloud predictor.

### Optional RAFT correspondence preview

`point_flow_raft_preview.py` is an experimental alternative and publishes to
`/thesis/perception/point_flow_raft_preview`; it does not replace or publish on
the Farneback topic. Its RGB images use TorchVision RAFT small C_T_V2 with
both flow directions, followed by the same depth, confidence, velocity and
rendering pipeline as the Farneback preview. It requires an NVIDIA CUDA-enabled
PyTorch/TorchVision environment, compatible with the ROS Python libraries, and
an explicitly provisioned local C_T_V2 weights file. The node does not download
weights automatically. Set `THESIS_RAFT_WEIGHTS` to that file before starting.

The local setup uses `build/raft_env` and `build/raft_weights`; these are build
artifacts and may need to be recreated if that directory is cleaned. After
sourcing ROS and the workspace, start RAFT and its separate RViz display:

```bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python install/thesis_perception/lib/thesis_perception/point_flow_raft_preview.py
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/point_flow_raft_preview.rviz
```

The two previews can run independently and publish different topics, but
concurrent execution shares GPU and camera resources. RAFT has been exercised
with the recorded RGB-D sequence and the D435i, without ground truth; the model
is not validated for accuracy, full coverage, dropout response or safety. It
does not feed robot control or a safety supervisor. To return to the previous
preview, start `ros2 run thesis_perception point_flow_preview.py` and use the
`point_flow_preview.rviz` configuration described above.
