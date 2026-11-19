# Validación de modelos 3D y materiales en Gazebo/RViz

Fecha: 2026-11-09; cierre final de regresión: 2026-11-10

## Alcance

Se corrigió la presentación en simulación de UR17g, TM25S y P3020 y se
robustecieron sus recursos temporales y la selección segura del launch. No se
modificaron controladores, límites, cinemática, colisiones, cápsulas ni la
ruta histórica del JACO2. No se utilizó hardware físico.

## Cambios validados

- El `robot_description` conserva URI `package://` válidas para RViz y Gazebo.
- La resolución a rutas locales se realiza solo en una copia interna usada
  para calcular las cápsulas de seguridad.
- Gazebo recibe los directorios de recursos sin reemplazar el entorno previo.
- UR17g conserva los materiales embebidos en sus DAE oficiales.
- TM25S deja de imponer verde esmeralda y usa los OBJ/MTL oficiales. Sus
  8.741 bloques de material redundantes se consolidan en 28 grupos efectivos
  sin cambiar ninguna de sus 267.898 caras ni su asignación a materiales.
- P3020 conserva sus 22 colisiones y aplica la paleta oficial azul/blanca a
  sus 22 visuales mediante materiales explícitos y únicos.
- RViz usa QoS `Transient Local` para `robot_description`.
- Las cápsulas `ALLOW` usan alpha `0.38` y no se dibujan dos veces.
- Las URI `file://` decodifican rutas con espacios y rechazan hosts remotos.
- Las copias renderizables del TM25S se publican atómicamente en una única
  caché reutilizable. El directorio de ejecución generado se elimina al
  apagar el launch; la prueba viva confirmó que no quedan directorios
  `/tmp/thesis_cell_*` acumulados.
- `historical_robot.launch.py` usa JACO2 como valor predeterminado. El launch
  interno rechaza `jaco2 + historical_stack` para impedir que un operador
  omita accidentalmente la pila histórica exacta.

## Validación estática

La conversión real `ign sdf -p` terminó sin errores:

| Perfil | Visuales URDF | Visuales SDF | Resultado de materiales |
| --- | ---: | ---: | --- |
| UR17g | 8 | 8 | DAE oficiales preservados |
| TM25S | 7 | 7 | OBJ/MTL oficiales, agrupación redundante consolidada |
| P3020 | 22 | 22 | 20 azul marino, 2 blancos, 0 negros accidentales |

Todas las mallas publicadas usan `package://` o `file://` y resuelven a un
archivo existente. Las cápsulas continúan derivándose de las mismas
geometrías de colisión: UR17g 8, TM25S 7 y P3020 22.

## Compilación y pruebas

Se compilaron con `--symlink-install` solamente los paquetes afectados:

```text
thesis_core
thesis_multi_robot
```

La regresión conjunta posterior cubrió también la interfaz y simulación:

```text
398 tests, 0 errors, 0 failures, 3 skipped
```

Desglose:

| Paquete | Aprobadas | Omitidas |
| --- | ---: | ---: |
| thesis_core | 198 | 1 |
| thesis_ui | 31 | 1 |
| thesis_simulation | 53 | 1 |
| thesis_multi_robot | 113 | 0 |

## Validación viva independiente

Cada brazo se ejecutó por separado. Se ordenó un incremento simulado de
`0.01 rad` y se comprobó readiness, FK-TF, supervisión, proyección futura y
resultado del controlador.

| Brazo | READY | FK-TF | Decisión | Ejecución | Predicción |
| --- | --- | --- | --- | --- | --- |
| JACO2 | Sí | error 0.000000 m | ALLOW/CLEAR | SUCCEEDED | 21 muestras |
| UR17g | Sí | error 0.000000 m | ALLOW/CLEAR | SUCCEEDED | 21 muestras |
| TM25S | Sí | error 0.000000 m | ALLOW/CLEAR | SUCCEEDED | 21 muestras |
| P3020 | Sí | error 0.000000 m | ALLOW/CLEAR | SUCCEEDED | 21 muestras |

Después de introducir la caché final, TM25S se volvió a ejecutar y mover con
`0.01 rad`. El resultado fue `READY`, FK-TF `0.000000 m`, `ALLOW/CLEAR`,
`PENDING -> ACCEPTED -> SUCCEEDED` y 21 muestras de predicción. El cierre fue
limpio y dejó solo una caché acotada de 35 MB, no una copia por lanzamiento.

Gazebo creó los tres perfiles añadidos sin errores de malla o recursos. RViz
se inició para UR17g, TM25S y P3020 con OpenGL 4.6; los tres registros quedaron
sin `Could not load resource`, errores de malla ni excepciones de RobotModel.

Las capturas finales confirman `RobotModel/Status: Ok`, el modelo físico bajo
las cápsulas transparentes, los volúmenes futuros y la apariencia del
fabricante en Gazebo. También se guardó una referencia nueva de la ruta
histórica JACO2 para comparar los cuatro brazos:

| Brazo | RViz | Gazebo |
| --- | --- | --- |
| JACO2 | `jaco2_rviz_reference.png` | `jaco2_gazebo_reference.png` |
| UR17g | `ur17g_rviz.png` | `ur17g_gazebo_full.png` |
| TM25S | `tm25s_rviz_final.png`, `tm25s_rviz_robotmodel_only.png` | `tm25s_gazebo_final_close.png` |
| P3020 | `p3020_rviz.png` | `p3020_gazebo_close.png` |

`tm25s_rviz_robotmodel_only.png` desactiva temporalmente los displays de
cápsulas y obstáculo para demostrar que la geometría visible pertenece al
`RobotModel` físico; `tm25s_rviz_final.png` vuelve a mostrar la misma malla
con la proyección preventiva completa.

## Entorno gráfico

La terminal integrada de VS Code Snap inyecta bibliotecas incompatibles y
puede producir `libpthread.so.0: GLIBC_PRIVATE`. La aplicación funcionó al
sanear ese entorno sin modificar el sistema. Para operación normal se debe
usar una terminal nativa abierta con `Ctrl+Alt+T`:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_historical_robot
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=p3020
```

Cambiar únicamente `robot:=p3020` por `ur17g`, `tm25s` o `jaco2`; ejecutar un
solo brazo por vez.

## Observaciones

En algunos cierres manuales con `Ctrl+C`, un nodo Python o `parameter_bridge`
puede emitir un mensaje transitorio durante la destrucción de recursos. Ocurre
después de completar la validación y no dejó procesos activos. No se
observaron errores durante arranque, supervisión, proyección o movimiento
simulado.
