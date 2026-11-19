# Validación del rediseño de interfaz JACO2

Fecha de campaña: 2026-11-12, según contexto de fecha del usuario.
Alcance: escritorio y simulación offline. No se inició hardware Kinova.

## Resultado implementado

- Nuevo diseño predeterminado de `joint_gui`, maximizado y con paleta cálida.
- Navegación por tareas, barra persistente de disponibilidad/estado y páginas
  independientes para resumen, articular/manual, cartesiano, trayectoria,
  diagnóstico e historial.
- Diagnóstico e historial se pueden abrir en ventanas auxiliares reutilizando
  los mismos widgets; su cierre restaura la página sin apagar ROS.
- F11/Esc para pantalla completa y retorno a maximizado.
- Distribución adaptable, sin desplazamiento horizontal en las páginas probadas
  a 1366×768 y 1920×1080. La página articular conserva desplazamiento vertical.
- `ui_layout:=legacy` conserva la presentación anterior para comparación.
- Nuevo `multiarm_gui` de observación, con seis suscripciones en un nodo propio:
  cuatro estados articulares, disponibilidad híbrida y estado global. No añade
  suscripciones al ejecutor de control monobrazo ni escribe órdenes.

La ventana de control reutiliza los mismos botones, señales, publicadores y
temporizadores. No se alteraron los paquetes de control, supervisión, geometría,
hardware, mensajes, configuración multi-brazo ni launches durante esta tarea.
Se preservaron todos los cambios anteriores del workspace.

## Referencia y pruebas automáticas

Resultado final: **102 pruebas aprobadas en 1.87 s**, sin fallos. Incluye 45
pruebas de GUI/layout/observador y 57 de los contratos multi-brazo seleccionados.
Compilación final: **1 paquete (`thesis_ui`) terminado correctamente**.
Flake8 de los cinco módulos/pruebas nuevos y `git diff --check`: correctos.

La referencia anterior a la implementación pasó 25 pruebas de GUI y las suites
focales de seguridad/disponibilidad multi-brazo. La validación final utiliza:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  -p no:cacheprovider \
  src/thesis_ui/test/test_gui_state.py \
  src/thesis_ui/test/test_workspace_layout.py \
  src/thesis_ui/test/test_multiarm_observer.py \
  src/thesis_simulation/test/test_global_safety_gate.py \
  src/thesis_simulation/test/test_adapter_global_watchdog.py \
  src/thesis_simulation/test/test_hybrid_readiness_contract.py \
  src/thesis_simulation/test/test_hybrid_scenario.py \
  src/thesis_core/test/test_multiarm_runtime.py \
  src/thesis_core/test/test_multiarm_coordination.py \
  src/thesis_core/test/test_multiarm_ros_contract.py \
  src/thesis_simulation/test/test_multiarm_config.py \
  src/thesis_simulation/test/test_multiarm_safety_contract.py
```

Cobertura específica nueva: navegación sin publicaciones, una sola instancia de
los controles, bloqueos durante ejecución, HOLD/cero al salir de JOG, no
reanudación al regresar o recuperar foco, conservación de órdenes pendientes,
equivalencia de payloads entre legacy/workspace, cierre de auxiliares sin cierre
ROS, cierre principal con dos auxiliares abiertas e idempotencia, adaptación de
ancho, aislamiento de observaciones por brazo y datos inválidos/vencidos.

La comparación de mensajes articulares, cartesianos y de trayectoria usa puertos
capturadores en pruebas; no demuestra por sí sola ejecución de todos esos modos
en Gazebo ni ejecución física.

Compilación limitada a `thesis_ui`, mediante `colcon build --packages-select
thesis_ui --symlink-install`. Los nuevos módulos y pruebas pasan flake8 con
longitud máxima de 99 caracteres. El código heredado no fue reformateado.

## Integración con cuatro identidades en Gazebo

Se arrancó el launch existente en un dominio aislado:

```bash
ROS_DOMAIN_ID=81 IGN_PARTITION=codex_ui_review_20261112 \
  ros2 launch thesis_simulation hybrid_three_sim.launch.py \
  headless:=true simulation_output_enabled:=false
```

El tablero Qt offscreen recibió estados recientes de `arm_01`, `arm_02`,
`arm_03` y `arm_04`, disponibilidad `READY` y estado global `ALLOW` con distancia
informada de 1.852473157274673 m. Se inspeccionaron los publicadores del nodo de
observación: solamente `/parameter_events` y `/rosout`, ambos infraestructura
de ROS; ninguno de mando, estados articulares o hardware. El launch acabó
limpiamente al detener el proceso de prueba exacto.

Este ensayo comprueba lectura y aislamiento del tablero. No es una campaña de
movimiento simultáneo de los tres actuadores. `arm_01` sigue siendo sintético.

## Comparación de movimiento monobrazo

Se inició el pipeline monobrazo existente, sin GUI automática ni RViz:

```bash
ROS_DOMAIN_ID=82 IGN_PARTITION=codex_ui_single_20261112 \
  ros2 launch thesis_simulation thesis_system.launch.py \
  headless:=true start_gui:=false start_rviz:=false \
  simulation_output_enabled:=true use_demo_obstacle:=true
```

Con la física original de 1 ms se observaron alternancias de proximidad
`TF_UNAVAILABLE`. Una solicitud desde legacy fue rechazada con STOP por esa
causa. También se observó ausencia de ejecución en el primer intento workspace.
No se relajaron interlocks ni se publicaron órdenes por fuera del supervisor.

Para disponer de una condición de prueba estable se cambió únicamente la
simulación lanzada por esta campaña a pasos de 5 ms, reduciendo la carga de
reloj/TF. Es el mismo tamaño de paso ya usado por el mundo híbrido. No se editó
el mundo monobrazo ni su configuración persistente:

```bash
IGN_PARTITION=codex_ui_single_20261112 ign service \
  -s /world/jaco_world/set_physics \
  --reqtype ignition.msgs.Physics --reptype ignition.msgs.Boolean \
  --timeout 3000 --req 'max_step_size: 0.005 real_time_factor: 1.0'
```

El servicio devolvió `data: true`. Bajo esta condición, cada presentación se
conectó por separado al mismo pipeline, esperó READY, copió la postura medida y
solicitó J1 +1° con los controles existentes y una duración de 3 s:

| Presentación | Decisión | Ejecución | Desplazamiento medido J1 |
| --- | --- | --- | --- |
| Legacy | ALLOW / CLEAR | SUCCEEDED | 0.016755158168588257 rad |
| Workspace | ALLOW / CLEAR | SUCCEEDED | 0.016755160818912894 rad |

La diferencia entre desplazamientos medidos fue aproximadamente 2.65e-9 rad.
No se interpreta como precisión física del brazo; corresponde a simulación.

Además se probó JOG articular en workspace, navegación a diagnóstico para
solicitar HOLD, apertura/cierre de la ventana auxiliar y regreso a articular:
desplazamiento 0.011894888738366659 rad, deriva posterior al HOLD de
1.468971291829213e-9 rad, modo JOG desactivado y sin reanudación automática.
El proceso de simulación se detuvo al finalizar; el ajuste temporal de física
desapareció con ese proceso.

## Evidencia visual

- `2026-11-12_workspace_overview_1920.png`: resumen final con datos simulados
  mediante FakeNode, no una captura de hardware.
- `2026-11-12_workspace_joints_1920.png`: controles finales, 1920×1080, FakeNode.
- `2026-11-12_workspace_overview_1366.png` y
  `2026-11-12_workspace_joints_1366.png`: distribución final estrecha, FakeNode.
- `2026-11-12_dashboard_live_1280.png`: observación conectada a Gazebo.
- Las imágenes `ui_articular` y `ui_final_*` de esta fecha registran iteraciones
  intermedias; las capturas `workspace_*` representan la presentación final.

## Límites y uso

La compatibilidad funcional de la GUI monobrazo queda respaldada por regresiones
y una comparación real de objetivo articular en Gazebo bajo la condición
temporal indicada. No se afirma validación física ni ausencia absoluta de fallos.
La intermitencia TF del pipeline monobrazo a 1 ms existía con la presentación
legacy y requiere un diagnóstico aparte si aparece en la máquina del operador.

El nuevo tablero no ofrece mando multi-brazo. El launch híbrido no incluye toda
la ruta preventiva local requerida para transformar una intención de GUI en
`candidate_supervised_*`; conectar un botón directamente a la puerta global
saltaría esa frontera. No se introdujo ese bypass ni se sustituyó el flujo
actual de los cuatro brazos. Instrucciones de uso en `src/thesis_ui/README.md`.
