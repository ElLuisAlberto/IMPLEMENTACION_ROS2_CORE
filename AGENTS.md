# Guía para trabajar en este proyecto

## Contexto y autonomía

- Antes de cambiar código, inspecciona el contexto relevante: arquitectura y documentación relacionadas, flujo completo afectado, implementaciones existentes y pruebas. Sigue las llamadas entre paquetes cuando el comportamiento cruce sus límites; no te limites al archivo señalado si la causa está en otra parte.
- Basa el diagnóstico en el código y en evidencia reproducible. Corrige la causa raíz con los cambios necesarios y coherentes, siguiendo los patrones del proyecto. Puedes editar, compilar y probar sin pedir autorización para cada paso.
- Amplía el alcance cuando sea necesario para que la solución quede completa y consistente; evita cambios ajenos al problema. Si hay varias interpretaciones con consecuencias importantes, pregunta antes de elegir.
- Revisa los cambios existentes antes de editar. Consérvalos e intégralos; no descartes ni sobrescribas trabajo ajeno.
- Trabaja primero en simulación. No actives salida ni ordenes movimiento al JACO2 físico sin confirmar que el usuario es operador autorizado, el área está despejada y la parada de emergencia es accesible.
- Antes de la primera prueba física, verifica conexión, diagnóstico, armado y disponibilidad sin mover el brazo. Para cada etapa que pueda moverlo, comienza con desplazamientos mínimos y velocidad reducida, y solicita confirmación del operador.
- No uses `sudo` ni instales paquetes sin autorización. Si una aplicación gráfica falla por el entorno de la terminal integrada, diagnostica primero sin modificar el sistema; pide usar una terminal nativa solo si es imprescindible.

## Validación y comunicación

- Tras los cambios, compila solo los paquetes afectados y ejecuta las pruebas pertinentes. Comprueba el resultado real y comunica con claridad cualquier fallo o validación que no haya sido posible.
- No crees informes o archivos de planificación rutinarios. Al terminar, da un resumen breve de la causa, los cambios y la validación; crea informes solo si el usuario los pide o la tarea lo requiere.
- Si se solicita un informe, no lo guardes en la raíz: diagnósticos en `docs/codex_reports/` y resultados de pruebas en `evidence/codex_reports/`. Crea la carpeta correspondiente si hace falta y no sobrescribas informes existentes.

## Observación de ROS 2

- Para iniciar la D435i y RViz, usa el procedimiento de `thesis_perception/README.md`: el launch aplica los parámetros de cámara y solicita la RTX para RViz. La superposición guardada es experimental y debe habilitarse explícitamente; nunca es una calibración ni inicia el control físico.
- Para consultar nodos o tópicos con el CLI `ros2`, usa `--no-daemon --spin-time 2` para evitar el daemon de `ros2cli`.
- Ejecuta cada cliente de observación por separado y espera a que termine antes de iniciar el siguiente.
- `tf2_echo` no acepta esas opciones; si hay que detenerlo, detén solo ese cliente, nunca el proceso de lanzamiento.

## Abrir la simulación actual de un brazo

Cuando el usuario pida abrir Gazebo, RViz y la interfaz del brazo simulado,
usa este procedimiento y la documentación vigente del proyecto. No inicies
ninguna simulación automáticamente al comenzar un chat. Si no queda claro qué
brazo desea, pregunta; `jaco2`, `ur17g`, `p3020` y `cs625` son perfiles
distintos. Esta sección no autoriza iniciar el brazo físico.

- Comprueba primero los procesos y nodos activos en el escritorio real, no
  solo dentro de un sandbox que pueda ocultarlos. Evita duplicar lanzamientos,
  relojes, controladores y tópicos. Si el usuario pide cambiar de brazo, cierra
  ordenadamente solo la sesión simulada pertinente; conserva los procesos
  ajenos. Antes de reutilizar un dominio, comprueba que la sesión anterior
  y sus procesos hijos hayan terminado.
- Usa la raíz `/home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE`,
  `/opt/ros/humble/setup.bash` y el `install/setup.bash` de esta raíz. Evita
  `install_migracion`, lanzamientos provisionales o históricos distintos y
  ejecutables sueltos cuando se solicite la simulación completa.
- Si los cambios relevantes en `src/` aún no están compilados en `install/`,
  compila únicamente los paquetes afectados con `colcon build --packages-select`
  y carga de nuevo `install/setup.bash`. Conserva los cambios existentes y no
  instales paquetes ni uses `sudo`.
- Para una sesión individual usa el dominio y partición documentados, siempre
  que estén libres; si no, elige otros aislados y mantén los mismos valores en
  el launch y los clientes de observación. Ejecuta uno de estos comandos:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_historical_robot
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=jaco2
# O, en vez del anterior, selecciona exactamente uno de estos perfiles:
# ros2 launch thesis_multi_robot historical_robot.launch.py robot:=ur17g
# ros2 launch thesis_multi_robot historical_robot.launch.py robot:=p3020
# ros2 launch thesis_multi_robot historical_robot.launch.py robot:=cs625
```

El launch abre Gazebo, RViz y la GUI de `thesis_ui`. `jaco2` deriva a
`thesis_simulation/thesis_system.launch.py`; los otros tres usan la ruta
individual de `thesis_multi_robot/cell.launch.py` con su namespace. El mundo
común de los lanzamientos individuales es
`src/thesis_simulation/worlds/jaco2_workstation.sdf`: laboratorio académico y
mesa simulada de 2,40 × 1,60 m con cuatro patas. RViz muestra la envolvente,
el horizonte, la mesa y el obstáculo virtual; la envolvente usada por el
supervisor es experimental y no acredita seguridad física.
Los perfiles `ur17g`, `p3020` y `cs625` incorporan automáticamente el CAD
instrumentado estático a colores, mediante
`sistema_gripper_instrumentado_colores.dae`; el STL original se conserva para
la envolvente. El `jaco2` conserva su propia pinza Kinova y no lleva este CAD.

Si una ventana falla por contaminación de bibliotecas de Snap, diagnostica el
entorno e inicia el mismo launch con un entorno gráfico limpio conservando los
valores reales de `DISPLAY`, `XAUTHORITY`, `XDG_RUNTIME_DIR` y, si corresponde,
`DBUS_SESSION_BUS_ADDRESS`. No supongas que sus valores son iguales a los de
una sesión anterior ni modifiques el sistema. No abras nodos o RViz adicionales
si el launch ya los inicia.

Tras iniciar, comprueba los nodos y el estado de preparación del brazo con
clientes separados y las opciones de observación indicadas arriba. Para JACO2
el estado está en `/thesis/system_readiness`; para los demás está en
`/<perfil>/thesis/system_readiness`. Comprueba también que RViz y la envolvente
estén publicándose. Comunica si hay `WARNING`, `STOP`, nodos duplicados o una
ventana que no se pudo abrir; no declares lista una sesión sin comprobarla.

## Configuración de percepción que el usuario quiere recuperar

Cuando el usuario pida volver a abrir «la configuración de RAFT que funcionaba»
o «lo que estaba abierto», usa esta referencia de la sesión del 23 de noviembre
de 2026. Comprueba primero qué procesos ya están activos para no duplicarlos;
no la arranques automáticamente al iniciar un chat. Es una visualización de
percepción experimental y no inicia control ni movimiento del JACO2.

La configuración activa y reproducible más reciente es D435i → comparación
RGB-D en Python/CPU → RAFT experimental CUDA en GPU → `raft_lines_node` C++
en CPU → RViz con PRIME NVIDIA. Conserva 12 iteraciones,
alcance radial máximo de 1,2 m, filtro de mediana y confirmación temporal en dos
imágenes. Usa el filtro adaptativo experimental: 15 puntos de soporte para
grupos grandes y al menos 5 con velocidad coherente para grupos pequeños.
La confirmación temporal usa la búsqueda 3×3 con comprobaciones de profundidad
y velocidad (`temporal_search_3x3:=true`); su eficacia en vivo aún no se ha
validado de forma concluyente.
El barrido es de líneas cian finas y transparentes, no cubos. Es una prueba;
no se ha validado su precisión con obstáculos pequeños ni como protección.
En el estado activo usado en esta sesión, `async_pairing:=true`.
El predictor y el marcador se remapean a los tópicos de la vista para mantener
el mismo RViz.

Asignación actual:

| Proceso | Ejecución | Script o ejecutable | Tópicos principales |
| --- | --- | --- | --- |
| D435i | CPU / Intel RealSense | `realsense2_camera_node` | `/camera/d435i/color/image_raw`, `/camera/d435i/aligned_depth_to_color/image_raw` |
| Comparación RGB-D | CPU / Python-OpenCV | `distance_comparison_views.py` | `/thesis/comparacion/distancia`, `/thesis/comparacion/suavizado` |
| RAFT experimental | GPU NVIDIA / PyTorch CUDA | `point_flow_raft_preview_experimental.py` | `/thesis/perception/predicted_points` |
| Barrido C++ | CPU | `raft_lines_node` | `/thesis/perception/swept_volume` |
| RViz | GPU NVIDIA mediante PRIME | `rviz2` | suscripciones visuales guardadas |

Desde la raíz del proyecto, con la D435i conectada y las dependencias y los
artefactos locales ya disponibles, inicia cada comando en una terminal o
proceso separado. En cada terminal carga primero:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. Cámara sin preprocesador, nodo de salud, predictores adicionales ni RViz:

```bash
ros2 launch thesis_perception d435i_standalone.launch.py \
  start_rviz:=false start_processing:=false start_extractor:=false \
  start_cloud_prediction:=false start_distance_comparison:=true
```

2. Generador de líneas C++:

```bash
ros2 run thesis_perception raft_lines_node --ros-args \
  -r /thesis/perception/raft_swept_surface_experimental:=/thesis/perception/swept_volume
```

3. Predictor RAFT experimental:

```bash
THESIS_RAFT_WEIGHTS="$PWD/build/raft_weights/raft_small_C_T_V2-01064c6d.pth" \
  build/raft_env/bin/python \
  install/thesis_perception/lib/thesis_perception/point_flow_raft_preview_experimental.py \
  --ros-args -p profile_timing:=true -p sweep_mode:=cpp -p async_pairing:=true \
  -p max_valid_range_m:=1.2 -p temporal_motion_confirm:=true -p adaptive_small_motion:=true \
  -p temporal_search_3x3:=true \
  -r /thesis/perception/point_flow_raft_preview_experimental:=/thesis/perception/predicted_points
```

4. RViz con la vista que estaba abierta y la RTX solicitada:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d install/thesis_perception/share/thesis_perception/rviz/raft_adaptive_preview.rviz
```

Verifica con `ros2 node list --no-daemon --spin-time 2`: deben estar la cámara,
`/comparacion_distancia_suavizado`, `/point_flow_raft_preview_experimental`,
`/raft_lines_experimental` y `/rviz`.
Los transform listeners de RViz pueden tener nombres variables. En esta vista,
`/pointcloud_preprocessor`, `/perception_health`, `/cloud_prediction`, el RAFT
legacy y `/raft_sweep` no deben estar activos.
Los códigos anteriores se conservan bajo `thesis_perception/scripts/alternativas/`
y `thesis_perception/src/alternativas/`; algunos informes históricos pueden
mencionar sus rutas anteriores. `scripts/point_flow_preview.py` permanece en su
ubicación porque el predictor actual lo importa.
No inicies instancias duplicadas ni envíes datos de pruebas a los mismos
tópicos de la sesión activa. Para detenerla, cierra cada uno de sus cinco
procesos de forma ordenada; no cierres procesos ajenos. Los pesos y el entorno
`build/raft_env` son artefactos locales: comprueba que existan antes de iniciar;
si faltan, no instales ni descargues nada sin autorización.
