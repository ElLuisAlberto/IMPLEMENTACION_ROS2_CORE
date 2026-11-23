# TESIS_2_IMPLEMENTATION

Plataforma experimental en ROS 2 para supervisar comandos de movimiento de un
manipulador Kinova Jaco2 antes de enviarlos a un backend de ejecución.

La propuesta separa la intención del usuario, la cinemática, la predicción de
corto horizonte, la evaluación geométrica, la decisión de seguridad y el
adaptador de ejecución. Esto permite estudiar la misma política en simulación
y, de forma controlada, frente al brazo físico.

## Guía rápida

- [Arquitectura y paquetes](#arquitectura): dónde están el supervisor, la GUI y
  los adaptadores.
- [Modos de trabajo](#modos-de-trabajo) y [preparación física](#preparación-física):
  alcance y condiciones de cada modo.
- [Comandos de ejecución](#comandos-de-ejecución): simulación, observación y
  apertura de la interfaz física, incluido el diagnóstico de armado.
- [Documentación técnica](docs/implementation_notes/): contratos e
  implementación; [informes](docs/codex_reports/) y
  [evidencias](evidence/codex_reports/) conservan el historial de pruebas.

Los informes describen el estado y las condiciones de su fecha; para arrancar
el sistema, consulta primero los comandos actuales de este README. Algunas
notas y herramientas antiguas todavía usan la ruta
`~/Escritorio/TESIS_2_IMPLEMENTATION`; revisa sus rutas y requisitos antes de
ejecutarlas en este workspace.

## Estado de la tesis

Estado al 22/10/2026: **simulación y movimiento articular físico J1 validados;
percepción RGB-D en observación, todavía fuera de la decisión preventiva**.

- El JACO2 funciona mediante la ruta candidato → supervisor preventivo →
  adaptador Kinova. El backend USB, feedback a 20 Hz, watchdog, TF y retorno a
  `hold` fueron comprobados físicamente.
- Dos ciclos J1 de +3° y retorno al origen terminaron dentro de 0.5° de error,
  sin watchdog, órdenes obsoletas ni movimiento durante `hold`.
- El armado inicia sin conservar JOG anteriores. Un STOP geométrico permite
  únicamente una retirada supervisada y no desarma por un STOP histórico.
- Existe una esfera virtual reproducible con escenarios `PARK`, `BLOCK_START`
  e `INSERT_STOP`. PARK está calibrado aproximadamente a 0.40 m; los escenarios
  STOP aún requieren su prueba física separada.
- La Intel RealSense D435i entrega RGB, profundidad y nube. Se prepararon un
  preprocesamiento de corto alcance y filtros sombra de cápsulas y mallas para
  distinguir el brazo de posibles puntos externos, sin conectarlos al
  supervisor ni habilitar salida física.
- Las capturas sincronizadas y la cadena TF de cámara se verificaron, pero el
  ajuste extrínseco basado en bordes se rechazó: se alineaba con el fondo y
  el robot rojo. Hay un tablero ChArUco y un detector offline preparados para
  una calibración temporal con referencia física; faltan su montaje y pose
  medida respecto de `world`.
- La ruta cartesiana física y la integración RGB-D → obstáculos siguen
  pendientes. Por ello no se declara terminado el Avance 3.

La evidencia técnica está organizada en
[`docs/codex_reports/`](docs/codex_reports/) y
[`evidence/codex_reports/`](evidence/codex_reports/).

La ampliación experimental en simulación con **UR17g-1300 y P3020**
junto al JACO2 está documentada en
[`src/thesis_multi_robot/README.md`](src/thesis_multi_robot/README.md).
Usa perfiles URDF y namespaces propios en Gazebo Fortress. Además de la célula
nominal, existe una ruta individual de paridad que reutiliza literalmente la
GUI histórica, el supervisor preventivo y la proyección futura de RViz para
UR17g y P3020. El perfil `jaco2` del mismo comando deriva al launch
histórico intacto para no cambiar su comportamiento ni su geometría.

### Misma interfaz histórica para los tres brazos simulados

Preparar cada terminal desde la raíz real de este repositorio:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_historical_robot
```

Ejecutar **solo una** de estas órdenes por vez:

```bash
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=jaco2
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=ur17g
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=p3020
```

Cada orden abre Gazebo, la misma GUI de `thesis_ui` y RViz. Para una prueba
automatizada sin ventanas se pueden añadir
`headless:=true start_gui:=false start_rviz:=false`. P3020 conserva los mismos
controles, pero sus cinco ejes solo garantizan IK de posición; una orientación
explícita no alcanzable se rechaza de forma segura.

### Mesa y esfera de la estación

Los cuatro lanzamientos individuales (`robot:=jaco2`, `ur17g`, `p3020` y
`cs625`) usan un tablero horizontal de 1.20 × 0.80 × 0.04 m, con superficie
a 0.75 m. La esfera de Gazebo tiene 0.12 m de radio y centro en
`world` (0.60, 0, 1.27) m: quedan 40 cm libres. RViz muestra el tablero en
el display de obstáculos. En los perfiles con incertidumbre, el marcador
esférico incluye también el margen de la medición.

La supervisión contrasta las cápsulas con la esfera y el tablero en cada
postura muestreada. El tablero incluye sus cantos y cara inferior. Las patas
y los soportes son geometría de Gazebo. La célula conjunta tiene una mesa por
base y una esfera compartida en coordenadas de mundo. Los soportes de montaje
de la célula separan las envolventes conservadoras del hombro y el tablero.

Para representar la misma estación **virtual** en el JACO2 físico en modo
observación, usar `virtual_workstation:=true` en
`jaco_physical_system.launch.py`. En ese marco la base está a z=0:
el tablero termina en z=0 y la esfera tiene centro en z=0.52 m.
El preset selecciona la esfera FIXED. Para usar una fuente externa se puede
indicar `use_demo_obstacle:=false`. Los escenarios PARK, BLOCK_START e
INSERT_STOP conservan sus poses calibradas cuando se seleccionan explícitamente.

## Arquitectura

```text
GUI o fuente de intención
        ↓
comando candidato
        ↓
cinemática / trayectoria / predicción
        ↓
evaluación geométrica
        ↓
supervisor: ALLOW · WARNING · REDUCTION · STOP
        ↓
adaptador de simulación o adaptador Kinova
        ↓
Gazebo o Jaco2 físico
```

La GUI no se comunica directamente con el hardware. En simulación publica al
adaptador de simulación; en hardware la única ruta de salida pasa por
`jaco_hardware_adapter` y la API Kinova.

El monitor de proximidad clasifica la separación actual como `ALLOW`,
`WARNING` o `STOP`; no existe una banda fija de 15 cm que ordene reducir la
velocidad. La escala JOG `REDUCTION` se decide por separado, evaluando el
horizonte predictivo y reduciendo solo si invade la reserva protectora.
Durante JOG, RViz diferencia el horizonte aprobado del volumen nominal que
activó una reducción: este último se muestra en naranja, expandido por el
umbral predictivo publicado por el supervisor. La etiqueta incluye el estado,
la escala, la separación nominal mínima y el umbral de esa misma decisión,
correlacionados por `command_id`; si no coinciden los datos recientes, el
volumen nominal de reducción no se dibuja. El horizonte azul se toma de
`/thesis/supervised_jog_command` (referencia aprobada de horizonte completo),
no de `/thesis/applied_jog_command`, que es el paso corto que el adaptador
envía al controlador.

En RViz, la cota naranja muestra continuamente la distancia actual mínima
entre las cápsulas del robot y la mesa. Las líneas auxiliares desplazan la
cota fuera de la silueta del brazo.

## Código principal

| Paquete o carpeta | Función |
| --- | --- |
| `src/thesis_interfaces` | Mensajes y contratos compartidos |
| `src/thesis_core` | Supervisor, predicción, cinemática y geometría |
| `src/thesis_simulation` | Gazebo y adaptadores de simulación |
| `src/thesis_hardware_bridge` | Adaptador ROS 2–Kinova |
| `src/thesis_hardware` | Launch físico, readiness y precondiciones |
| `src/thesis_ui` | Interfaz para simulación y hardware |
| `src/thesis_telemetry` | Registro de decisiones y ejecución |
| `src/thesis_validation` | Validadores y escenarios |
| `src/thesis_multi_robot` | Perfiles y lanzamientos para varios robots simulados |
| `src/thesis_description` | Modelo, descripción y configuración visual del JACO2 |
| `src/gripper_instrumentado_description`, `src/sistema_gripper_instrumentado_description` | Descripciones del gripper instrumentado |
| `src/thesis_jaco_perception` | Auto-filtrado diagnóstico de cápsulas y mallas, aislado del supervisor |
| `thesis_perception` | Cámara D435i, preprocesamiento y extracción de obstáculos |
| `tools/physical` | Preflight físico y protocolo articular |
| `tools/perception` | Capturas diagnósticas y análisis offline de calibración |
| `tools/realsense_d435i_test` | Pruebas independientes de la cámara |
| `tools/simulation`, `tools/validation`, `tools/hardware`, `tools/demos` | Utilidades de simulación, validación, hardware y demostración |
| `third_party` | Recursos y dependencias externas de robots |
| `docs`, `evidence` | Documentación e informes; resultados y registros de pruebas |

Los directorios `build/`, `install/` y `log/` son artefactos locales de
compilación y no forman parte del repositorio. `build_migracion/` e
`install_migracion/`, si existen, son áreas locales de compilación separadas;
consérvalas mientras puedan ser necesarias. El paquete ROS `thesis_perception`
está en la raíz, mientras que los demás paquetes principales están en `src/`.

## Modos de trabajo

| Modo | Interfaz | Estado |
| --- | --- | --- |
| Simulación | GUI en `operation_mode=simulation` | Disponible para pruebas |
| Hardware en lectura | GUI física sin armado | Validado |
| Movimiento articular físico | GUI → supervisor → Kinova | J1 ±3° validado |
| Movimiento cartesiano físico | Ruta cartesiana supervisada | Bloqueado por A-02b |
| Visualización RGB-D | D435i → RViz | Disponible |
| Auto-filtrado sombra JACO2 | D435i → cápsulas/mallas → diagnóstico | Experimental, sin uso preventivo |
| Extrínseca D435i → `world` | ChArUco sobre base del JACO2 | Pendiente de pose física medida y captura válida |
| RGB-D conectado al supervisor | Cámara → obstáculo → supervisor | Pendiente |

Las rutas deben ejecutarse por separado. No se deben iniciar simultáneamente
Gazebo y el adaptador físico en el mismo dominio ROS.

## Preparación física

El procedimiento de conexión, el observador de solo lectura y la primera
prueba articular están documentados en
[`tools/physical/README.md`](tools/physical/README.md).

La inspección estática puede ejecutarse sin brazo:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
python3 tools/physical/physical_preflight.py --static-only
```

Con el brazo conectado, el preflight debe confirmar USB, API Kinova,
JointState, frecuencia, TF, proximidad, readiness, supervisor, ausencia de
movimiento y `armed=false`. Si no devuelve `PASS`, no se habilita salida física.

El protocolo validado comienza con un solo joint, velocidad reducida,
desplazamiento pequeño, `ALLOW` previo, watchdog y parada física disponible.
No se deben improvisar trayectorias cartesianas, movimientos largos ni
automatización de evasión.

## Comandos de ejecución

Ejecuta primero esta preparación en cada terminal nueva:

```bash
cd ~/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Si VS Code fue instalado como Snap y aparece un conflicto con bibliotecas GTK o
`libpthread`, abre una terminal normal del sistema. No hacen falta `sudo` ni
paquetes adicionales.

### Simulación completa

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  simulation_output_enabled:=true \
  use_demo_obstacle:=true \
  start_gui:=true \
  start_rviz:=true
```

Abre Gazebo, la GUI y RViz sin utilizar el USB del brazo.

### JACO2 físico en observación

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=false \
  use_demo_obstacle:=true \
  demo_obstacle_scenario:=PARK \
  start_gui:=true \
  start_rviz:=true \
  allow_gui_arm_control:=false
```

Conecta el brazo y publica su estado real, pero no permite armar ni mover.

### JACO2 físico con movimiento supervisado

```bash
ros2 launch thesis_hardware jaco_physical_system.launch.py \
  mock_hardware:=false \
  hardware_output_enabled:=true \
  use_demo_obstacle:=true \
  demo_obstacle_scenario:=PARK \
  demo_insert_stop_active:=false \
  start_gui:=true \
  start_rviz:=true \
  allow_gui_arm_control:=true
```

Habilita el botón de armado, pero el launch siempre comienza con
`armed=false`, `mode=hold` y salida 0 Hz. Úsalo únicamente con el área despejada,
la parada de emergencia accesible y proximidad `ALLOW/READY`. No abras dos
launches físicos a la vez: el bloqueo USB admite un solo dueño.

Este comando inicia también la esfera virtual `PARK`. Es un escenario de
prueba: no detecta personas ni obstáculos reales. El operador autorizado debe
comprobar el área y la parada de emergencia; antes de cada etapa que pueda
mover el brazo, confirma la prueba y comienza con desplazamientos mínimos y
velocidad reducida.

#### Si el armado muestra NOT_READY o STOP/HOLD

Comprueba primero que no haya una sesión física ya abierta. Si la interfaz
existente se inició con `use_demo_obstacle:=false`, no basta con abrir otra
GUI: el monitor sigue sin recibir una esfera virtual. Para comprobar la fuente,
ejecuta en otra terminal, después de la preparación indicada arriba:

```bash
ros2 topic info /thesis/obstacle_input --verbose --no-daemon --spin-time 2
```

Si aparecen cero publicadores y se desea trabajar con el escenario virtual,
inicia una sola fuente en una terminal y mantenla abierta:

```bash
ros2 run thesis_core obstacle_demo_source --ros-args \
  --params-file install/thesis_core/share/thesis_core/config/virtual_obstacle_scenarios.yaml \
  -p scenario:=PARK
```

No inicies esta fuente si ya hay otra publicando: revisa la fuente existente.
Comprueba por separado la proximidad y la disponibilidad, esperando a que
termine cada comando antes de ejecutar el siguiente:

```bash
ros2 topic echo /thesis/proximity_status --once --no-daemon --spin-time 2
ros2 topic echo /thesis/system_readiness --once --no-daemon --spin-time 2
ros2 topic echo /thesis/hardware/diagnostics --once --no-daemon --spin-time 2
```

Para el escenario PARK, verifica geometría válida y `ALLOW`, `READY`, conexión
y estado de articulaciones válido. Si aparece `ARM interlock rejected: STOP or
HOLD is active` después de recuperar esas condiciones, puede haber quedado un
STOP del supervisor retenido. Con el brazo detenido, pulsa **DESARMAR SALIDA
FÍSICA** para restablecerlo y vuelve a comprobar el diagnóstico: debe indicar
`armed=false`, `safety_stop_active=false` y una tasa de salida de 0 Hz. La
acción de desarmar detiene cualquier ejecución activa. No restablezcas ni armes
si persiste un STOP real, hay datos inválidos o falta el supervisor; investiga la
causa. Un mensaje de fallo anterior puede permanecer visible hasta otra acción.

El control manual físico requiere además que la salida esté armada y que no
haya una trayectoria activa. Si el armado sigue rechazándose, revisa el motivo
actual y el diagnóstico antes de continuar.

### Reabrir solo la GUI física

```bash
ros2 run thesis_ui joint_gui --ros-args \
  -r __node:=joint_control_gui_node \
  -p use_sim_time:=false \
  -p require_system_readiness:=true \
  -p operation_mode:=hardware \
  -p allow_hardware_arm_control:=true
```

Úsalo cuando el stack físico ya está activo y únicamente se cerró la ventana.

### Reabrir solo el RViz del robot

```bash
rviz2 -d "$(ros2 pkg prefix --share thesis_description)/rviz/jaco.rviz"
```

Muestra el modelo medido, cápsulas, horizonte y esfera virtual sin iniciar otro
adaptador USB.

### RealSense D435i y RViz RGB-D

Para la configuración actual de la D435i, el filtro de proximidad y RViz con
RTX, consulta [el arranque de percepción](thesis_perception/README.md#standalone-mode).
La superposición nube-robot documentada allí es experimental y no inicia ni
controla el brazo físico.

Para reabrir la cámara y comparar la predicción de movimiento de obstáculos
con RAFT o con el método anterior, consulta el [arranque paso a paso de la
previsualización](thesis_perception/README.md#arranque-de-la-c%C3%A1mara-y-predicci%C3%B3n-experimental).
Es una visualización de percepción; no inicia la evasión ni el control físico.

El [registro de avances de percepción y pruebas](docs/codex_reports/2026-11-22_registro_avances_percepcion_y_pruebas.md)
reúne la auditoría, los resultados y los rosbags de la sesión. El [filtro
rápido experimental](tools/perception/experimental_arm_segmentation/README.md)
se compila e inicia por separado; el arranque de percepción no lo inicia.
Los archivos `.db3` no se guardan en Git y necesitan un respaldo separado.

La siguiente es una alternativa genérica sin esos ajustes del proyecto:

En una terminal inicia color, profundidad alineada, RGB-D y nube de puntos:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_rgbd:=true \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

En otra terminal abre la configuración de nube de puntos:

```bash
rviz2 \
  -d /opt/ros/humble/share/realsense2_camera/examples/pointcloud/rviz/pointcloud.rviz \
  --ros-args -r __node:=rviz2_rgbd
```

Los tópicos principales son `/camera/camera/color/image_raw`,
`/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/rgbd` y
`/camera/camera/depth/color/points`. Esta ejecución solo visualiza la cámara;
no la conecta todavía al supervisor preventivo.

Detén cualquier launch con `Ctrl+C`. Antes de reiniciar el stack físico,
confirma que la sesión anterior terminó para no competir por el USB.

## Percepción RGB-D

La cámara ya funciona como fuente RGB-D independiente y los filtros sombra
publican resultados diagnósticos en una sesión de sólo lectura. Esto **no**
demuestra que la nube externa esté libre de puntos del brazo ni autoriza usarla
para seguridad: persisten residuos sobre el brazo y no hay una extrínseca
validada. La prueba de alineación de silueta de [Fase 8T](docs/codex_reports/2026-10-22_fase8t_cadena_tf_y_candidato_extrinseco.md)
se rechazó. La [Fase 8U](docs/codex_reports/2026-10-22_fase8u_preparacion_charuco_temporal.md)
dejó preparado el tablero ChArUco; la captura siguiente espera la pose del
tablero medida por el operador.

El contrato de integración aún pendiente es:

```text
cámara → percepción → obstáculo válido → obstacle_input_bridge
→ proximity_monitor → safety_supervisor
```

Antes de declararla integrada faltan calibración cámara→`world` con referencia
física, validación del auto-filtrado y de la extracción de obstáculos, sellos de
adquisición, incertidumbre y pruebas físicas de `ALLOW`, `REDUCTION` y `STOP`.

## Verificación offline

La evidencia completa de las pruebas está en `evidence/codex_reports/`. Como
comprobación focal de la preparación física, se compilan solo los paquetes
afectados y se ejecutan los contratos de *readiness*, telemetría y bridge. Los
movimientos físicos no sustituyen una prueba unitaria.

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

colcon build --packages-select \
  thesis_core thesis_hardware_bridge thesis_hardware thesis_telemetry

python3 -m pytest -q \
  src/thesis_hardware/test/test_readiness_contract.py \
  src/thesis_hardware/test/test_readiness_logging.py \
  src/thesis_telemetry/test/test_telemetry_math.py

ctest --test-dir build/thesis_hardware_bridge \
  --output-on-failure -R '^test_bridge_logic$'
```

Las pruebas focales de la ruta cartesiana, JOG continuo, GUI, sincronización
FK-TF y escenarios de obstáculo virtual se ejecutan por separado:

```bash
python3 -m pytest -q \
  src/thesis_core/test/test_cartesian_kinematics.py \
  src/thesis_core/test/test_a02a_frame_tf_interlock.py \
  src/thesis_core/test/test_virtual_obstacle_scenarios.py \
  src/thesis_simulation/test/test_cartesian_jog_model.py \
  src/thesis_ui/test/test_gui_state.py
```

La prueba de contrato de percepción RGB-D permanece independiente del brazo:

```bash
python3 -m pytest -q \
  thesis_perception/test/test_perception_safety_contract.py
```

## Limitaciones y siguiente etapa

- El movimiento físico validado se limita a la ruta articular supervisada y a
  desplazamientos pequeños; no cubre todavía control cartesiano.
- A-02b debe definir y validar la coherencia TF-modelo antes de autorizar
  movimiento cartesiano.
- Los escenarios virtuales STOP están calibrados, pero su ensayo físico debe
  ejecutarse como una etapa separada y controlada.
- La RGB-D debe conectarse gradualmente al pipeline y permanecer fail-closed
  ante datos ausentes, vencidos o inconsistentes.
- Los filtros de cápsulas y mallas son diagnósticos de sesión, no una fuente de
  `obstacle_input` ni una garantía de espacio libre. La calibración ChArUco
  requiere primero imprimir, fijar y medir el tablero; no hay candidato activo.

El detalle de cada decisión, prueba y bloqueo se conserva en los informes; este
README resume el proyecto para presentación y consulta rápida.
