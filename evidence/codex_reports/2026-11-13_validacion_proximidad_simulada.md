# Validación de correcciones de proximidad en simulación

Fecha de cierre: 2026-11-13, Asia/Manila. Sin acceso ni movimiento físico.
Se preservaron los cambios previos del workspace. Los procesos de simulación
creados para esta campaña se detuvieron al terminar.

## Resultado

- Compilación correcta de los tres paquetes afectados: `thesis_core`,
  `thesis_simulation` y `thesis_ui`.
- **413 pruebas funcionales aprobadas**: core 241, simulación 118, UI 54.
- Flake8 de los módulos/pruebas/herramienta nuevos y `git diff --check` correctos.
- No se modificaron los radios conservadores del brazo, los umbrales de
  protección, el mundo de física de 1 ms ni las salidas del bridge físico.

## Reproducción anterior al arreglo

Se lanzó el monobrazo con selección explícita `topic` y demo fijo, en dominio
86 y partición Ignition aislada. Al trasladar únicamente la esfera de Gazebo
de X=0.6 a X=2.0 mediante `set_pose`, el monitor siguió evaluando X=0.6, con
clearance 0.375002 m y radio efectivo 0.14 m. El observador midió una discrepancia
de **1.4 m** entre esfera de escena y esfera evaluada. La aritmética FK contra
el objeto evaluado coincidía: el fallo era la selección de fuente.

Se comprobó además la pérdida del sello: el TFMessage producido por el bridge
original daba `stamp=(0,0)` para `safety_obstacle`; el Pose_V nativo tenía fecha
solo en el encabezado del vector. El monitor que dejaba de inventar datos lo
rechazó correctamente, exponiendo el fallo de integración antes oculto.

## Prueba monobrazo corregida

Lanzamiento utilizado, dominio 87 y partición aislada:

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  headless:=true start_gui:=false start_rviz:=false \
  simulation_output_enabled:=true
```

Se mantuvo el mundo original de 1 ms. El obstáculo ahora usa PosePublisher a
30 Hz y el bridge de poses individuales fechadas. No se volvió a fechar una
observación con el reloj de recepción. La GUI se conectó por separado con
tiempo simulado, disponibilidad requerida y fuente `gazebo`.

| Caso | Centro X / Y / Z, m | Resultado | Clearance medido, m |
| --- | --- | --- | ---: |
| Inicio | 0.60 / 0 / 0.65 | 99 ALLOW | 0.39500213411929286 |
| Alejado | 2.00 / 0 / 0.65 | 34 ALLOW | 1.7950006407896524 |
| Advertencia real | 0.45 / 0 / 0.65 | 32 WARNING | 0.24500284522520455 |
| Parada geométrica | 0.24 / 0 / 0.65 | 28 STOP geométricos | 0.035005334063812676 |
| Volver a alejar | 2.00 / 0 / 0.65 | 26 ALLOW | 1.7950006407896524 |

En las mediciones geométricas, el error FK contra la distancia publicada y la
discrepancia entre centro visible y evaluado fueron cero a la precisión del
observador. En la fase STOP hubo además **dos evaluaciones TF_UNAVAILABLE**:
se mantuvieron como fallos de disponibilidad, no como medidas de contacto.
No se afirma ausencia de todo fallo TF transitorio bajo carga.

Con la esfera alejada, se envió desde la GUI un objetivo J1 +1°, duración 3 s.
El supervisor decidió ALLOW y el adaptador devolvió **SUCCEEDED**. Desplazamiento
medido: 0.016638802903488303 rad. Esta prueba se completó con la física original,
sin el ajuste temporal a 5 ms que había sido necesario en la campaña de UI.

## Pérdida de fuente y presentación

Se detuvo únicamente el proceso de bridge de poses creado por esta campaña,
después de verificar su PID y argumentos. Durante el siguiente sondeo hubo
78 STOP / OBSTACLE_UNAVAILABLE, con `geometry_valid=false`, sin distancia
geométrica publicada como válida. Readiness pasó a WAITING por fuente vencida
y geometría no disponible.

Se abrió la GUI real offscreen contra ese pipeline: mostró
**«Sin geometría válida (datos)»**, conservó STOP y mantuvo el botón de envío
deshabilitado. Captura: `2026-11-13_proximidad_fuente_ausente.png`.

Los tests cubren además TF desconocido, valores no finitos, cero real de
contacto, distancias negativas, datos vencidos, ID JOG propio/ajeno y la retirada
protectiva que no debe habilitarse ante geometría desconocida.

## Escenario híbrido de cuatro identidades

Lanzamiento offline existente, dominio 88, salida simulada deshabilitada:

```bash
ros2 launch thesis_simulation hybrid_three_sim.launch.py \
  headless:=true simulation_output_enabled:=false
```

El sondeo de 8 s recibió estados articulares y snapshots geométricos válidos de
las cuatro identidades: `arm_01=synthetic_state`, `arm_02–04=gazebo`. Readiness
READY; **148 decisiones ALLOW/CLEAR**; mínimo estable **1.852473157274673 m**.
El cálculo independiente con las articulaciones recibidas coincidió en los seis
pares (los últimos estados tienen skew diagnóstico de 15 ms; no sustituyen la
sincronización del coordinador):

| Par | Separación, m |
| --- | ---: |
| 01–02 | 1.9112240826384777 |
| 01–03 | 1.9113203435596424 |
| 01–04 | 1.852473157274673 |
| 02–03 | 4.032640687119284 |
| 02–04 | 2.79 |
| 03–04 | 2.7651052158430693 |

Nueve regresiones adicionales contrastan bases, yaw, segmentos y efector contra
URDF expandido en dos configuraciones. No se probó mando de cuatro actuadores:
`arm_01` es sintético y el tablero multi-brazo continúa siendo observacional.

## Regresión del modo topic

Se lanzó explícitamente `obstacle_source_mode:=topic use_demo_obstacle:=true`
con radio 0.20 m, salida simulada deshabilitada, dominio 89. Hubo 96 WARNING
válidos y READY. Radio efectivo 0.22 m por incertidumbre de 0.02 m; clearance
0.2950021341192929 m, coincidente con FK. No apareció esfera Gazebo adicional.
Esto verifica que el modo explícito de estimación conserva su inflación de
incertidumbre y su representación independiente de una esfera física.

## Pruebas y reproducción

Compilación:

```bash
colcon build --packages-select thesis_core thesis_simulation thesis_ui \
  --symlink-install --event-handlers console_direct+
```

Las suites funcionales se ejecutaron por paquete, excluyendo los tres archivos
de copyright/flake8/pep257 heredados. Una invocación conjunta inicial encontró
colisión de nombres de esos archivos durante la colección de launch_testing;
se resolvió ejecutando los paquetes por separado, sin alterar sus tests.
Resultados finales: core 241/241, simulación 118/118, UI 54/54.

Se añadieron `test_proximity_source_validity.py`, `test_proximity_display.py`,
`test_simulation_scene_contract.py`, `test_proximity_readiness.py` y
`test_arm_geometry_alignment.py`. Los nuevos archivos pasan flake8 (99 columnas).

Observador reproducible, sin publicadores de mando ni clientes:

```bash
ROS_DOMAIN_ID=87 python3 tools/validation/probe_simulation_proximity.py \
  --mode single --duration 10
ROS_DOMAIN_ID=88 python3 tools/validation/probe_simulation_proximity.py \
  --mode hybrid --duration 10
```

Cada observador se ejecutó por separado. Sus únicos publicadores fueron la
infraestructura de ROS `/parameter_events` y `/rosout`. Todas las observaciones
son diagnósticas, no autorizaciones de movimiento.

## Límites

Las distancias son del modelo conservador de cápsulas, no de mallas visuales.
La esfera conserva el radio configurado al lanzar; la prueba interactiva validó
traslación, no redimensionamiento manual arbitrario. No se realizaron pruebas
físicas ni se declara terminado el control individual del escenario multi-brazo.

Diagnóstico: `docs/codex_reports/2026-11-13_diagnostico_falsa_proximidad_simulacion.md`.
