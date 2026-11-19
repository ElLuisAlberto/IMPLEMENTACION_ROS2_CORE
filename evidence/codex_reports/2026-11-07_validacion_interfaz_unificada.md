# Validación de la interfaz unificada multi-robot

Fecha: 2026-11-07

## Resultado final

**PASS en simulación.** No se usó hardware físico.

- Build selectivo: 9 paquetes terminados.
- Pruebas finales de `thesis_multi_robot`: 85/85, incluidas las regresiones
  terminal `ACCEPTED → SUCCEEDED` e inicialización stale→fresh de la UI.
- Lint de los siete archivos tocados/nuevos: 0 errores `ament_flake8`.
- PEP 257 de GUI, backend, launch y validador: 0 errores.
- Regresión offline histórica comunicada por auditoría:
  `thesis_core` 173/173 y `thesis_simulation` 41/41.

## Prueba viva de la GUI

`2026-11-07_multi_robot_gui_validation_v5/result.json` obtuvo `pass: true`:

- cuatro perfiles `FRESH/ALLOW`;
- grados de libertad 6/6/6/5;
- controladores de estado y trayectoria activos;
- JACO2, UR17g, TM25S y P3020 ejecutaron, uno por vez, `+0,02 rad` y retorno;
- no hubo cruce de IDs/comandos entre perfiles;
- el entry point Qt real permaneció activo en modo offscreen.

El argumento integrado `start_gui:=true` se probó con UR17g en
`2026-11-07_multi_robot_start_gui_launch_v2/launch.log`: la GUI arrancó,
ambos controladores se activaron y la GUI terminó limpiamente con `SIGINT`.
El intento anterior se conserva y permitió corregir un `KeyboardInterrupt`
dentro del temporizador Qt.

Los tres brazos añadidos también obtuvieron `pass: true` por separado:

- `2026-11-07_multi_robot_gui_ur17g/result.json`;
- `2026-11-07_multi_robot_gui_tm25s/result.json`;
- `2026-11-07_multi_robot_gui_p3020/result.json`.

Cada resultado incluye reloj ROS simulado, número de callbacks y ausencia de
errores de entrada.

## Regresión integral

`2026-11-07_multi_robot_cell_post_gui/all_result.json` obtuvo `pass: true`.
Para los cuatro perfiles verificó controladores, FK/TF, movimiento/retorno,
cancelación por obstáculo durante una acción, hold, rechazo por contacto,
rechazo de fuente caducada y recuperación. Cuatro órdenes concurrentes
alcanzaron error menor de 0,004 rad sin `STOP` de ejecución.

## JACO2 histórico

En `ROS_DOMAIN_ID=99` y una partición Gazebo aislada se inició el launch
histórico sin GUI, RViz ni hardware. Las consultas se ejecutaron por separado
con `--no-daemon --spin-time 2`:

- `/thesis/system_readiness`: `READY`;
- `/thesis/proximity_status`: `ALLOW`;
- margen mínimo: `1,2023092667 m`.

El log está en `2026-11-07_jaco2_historical_post_gui/launch.log`. El cierre
no dejó procesos ROS/Gazebo residuales.

## Intentos diagnósticos preservados

Las carpetas `2026-11-07_multi_robot_gui_validation` hasta `_v4` se conservan
como evidencia de desarrollo. Sus fallos no fueron ocultados: v1/v2 revelaron
reloj y finalización prematura; v3/v4 mostraron la carrera de carga simultánea
de `gz_ros2_control`. La ejecución v5 valida las correcciones.
