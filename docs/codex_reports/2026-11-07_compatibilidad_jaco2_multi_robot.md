# Diagnóstico de compatibilidad: JACO2 histórico y célula multi-robot

Fecha: 2026-11-07

## Hallazgo y corrección

La primera versión de `tools/simulation/build_multi_robot.sh` construía solo
las seis descripciones e integración de la célula nueva. Como consecuencia,
después de ejecutar `source install/setup.bash`, `ros2` no podía localizar
`thesis_simulation`; el código del JACO2 histórico no se modificó, pero su
paquete quedaba ausente del overlay generado.

El script se corrigió para construir e instalar también `thesis_core`,
`thesis_ui` y `thesis_simulation`. La reconstrucción selectiva terminó los
nueve paquetes correctamente. Tras ella, los prefijos de
`thesis_simulation`, `thesis_core`, `thesis_ui` y `thesis_multi_robot` se
resuelven desde el mismo overlay.

## No regresión del JACO2 histórico

No hay archivos modificados en los paquetes históricos de JACO2:
`thesis_core`, `thesis_description`, `thesis_simulation`, `thesis_ui`,
`thesis_hardware`, `thesis_hardware_bridge`, `thesis_telemetry` ni
`thesis_jaco_perception`. La integración reside en el nuevo paquete
`thesis_multi_robot`; `README.md` solo añade una referencia documental.

En `ROS_DOMAIN_ID=91` e `IGN_PARTITION=thesis_legacy_jaco_audit` se inició:

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  initial_pose:=z_ready use_demo_obstacle:=true obstacle_x:=2.0 \
  simulation_output_enabled:=true start_gui:=false start_rviz:=false \
  headless:=true
```

Se observaron por separado, con `--no-daemon --spin-time 2`:

- nodos históricos de supervisor, control, readiness y los tres adaptadores
  cartesianos;
- `/thesis/system_readiness = READY`;
- `/thesis/proximity_status = ALLOW`, con `minimum_clearance = 1.202309 m`.

No se enviaron comandos de movimiento ni se inició hardware físico. El cierre
por SIGINT no dejó procesos residuales. El bridge de reloj de `ros_gz_bridge`
terminó con `exit code -11` durante ese cierre forzado y `rclpy` mostró una
advertencia al destruir una tarea; ambos ocurrieron después de alcanzar
`READY`. Deben investigarse como robustez de apagado del stack histórico, pero
no son causados por un cambio de su código y no impidieron su arranque.

## Célula multi-robot con el overlay corregido

La validación completa posterior, en `ROS_DOMAIN_ID=88`, obtuvo `pass: true`
para JACO2, UR17g, TM25S y P3020. Confirmó controladores, TF/FK, movimientos
articulares mínimos y retorno, parada por obstáculo, datos caducados y cuatro
objetivos simultáneos. La evidencia está en:

`evidence/codex_reports/2026-11-07_multi_robot_compatibility_validation/`.

## Aislamiento obligatorio

No se deben iniciar `thesis_system.launch.py` y `cell.launch.py` con el mismo
`ROS_DOMAIN_ID` ni `IGN_PARTITION`. Ambos publican `/clock` y usan `world`;
mezclarlos contaminaría tiempo y TF aunque la célula nueva tenga controladores
namespaced. La documentación de la célula prescribe dominios 88 y 87,
respectivamente, como ejemplo de separación.

## Paridad funcional

Los tres perfiles nuevos no son aún equivalentes a todo el pipeline histórico
del JACO2. Comparten Gazebo, TF, controladores, límites, órdenes articulares
supervisadas y una IK puntual. Aún no comparten GUI/RViz de operación,
readiness integral, JOG cartesiano, trayectoria cartesiana recta/cancelable,
autocolisión, retiming de `REDUCTION`, percepción RGB-D/MoveIt ni coordinación
basada en objetivos futuros de pares. Por tanto, se aprueba la simulación
nominal multi-robot, no una afirmación de paridad completa.
