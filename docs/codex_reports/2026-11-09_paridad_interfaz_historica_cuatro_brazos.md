# Paridad de la interfaz histórica en cuatro brazos simulados

Fecha: 2026-11-09

## Resultado

La ruta individual de simulación quedó preparada para ejecutar JACO2,
UR17g-1300, TM25S o Doosan P3020 con la misma aplicación Qt histórica, la
misma familia de mensajes, el supervisor preventivo, la predicción temporal y
la proyección futura de RViz. Cada brazo se inicia por separado mediante
`historical_robot.launch.py`.

El JACO2 no pasa por la abstracción nueva: el wrapper deriva directamente a
`thesis_simulation/launch/thesis_system.launch.py`. Esta separación preserva
sus tópicos globales, sus funciones numéricas y el modelo histórico de seis
cápsulas. También sigue siendo válido iniciar ese launch original directamente.
El wrapper selecciona JACO2 por defecto y `cell.launch.py` rechaza el intento
de iniciar su pila histórica por la ruta genérica.

## Implementación

### Perfil cinemático y geométrico

- `thesis_core.robot_profile` valida articulaciones, límites, velocidades,
  frames, controlador, postura inicial y cápsulas desde el manifiesto.
- `thesis_core.robot_kinematics` calcula FK/IK desde el URDF para un número
  arbitrario de articulaciones.
- Supervisor, predicción articular, horizonte, swept volume, visualizador de
  cápsulas y monitor de proximidad consumen el mismo perfil.
- Sin manifiesto se conservan las constantes y funciones históricas del
  JACO2.

| Brazo | DOF activos | Cápsulas | Acción de trayectoria |
| --- | ---: | ---: | --- |
| JACO2 histórico | 6 | 6 | `/arm_controller/follow_joint_trajectory` |
| UR17g-1300 | 6 | 8 | `/ur17g/arm_controller/follow_joint_trajectory` |
| TM25S | 6 | 7 | `/tm25s/arm_controller/follow_joint_trajectory` |
| P3020 | 5 | 22 | `/p3020/arm_controller/follow_joint_trajectory` |

### Interfaz histórica

Se conserva el ejecutable `thesis_ui joint_gui` y la clase
`JointControlWindow`; no se construyó una GUI paralela para esta ruta. La tabla
articular, etiquetas, límites, velocidades, postura inicial y frames se
adaptan al perfil. Permanecen las pestañas, historial, objetivos articulares,
JOG, comandos cartesianos, readiness, decisiones y estados de ejecución.

Los indicadores de conexión resuelven el nombre final del tópico antes de
consultar publicadores, por lo que funcionan con los remappings namespaced.
Los perfiles no JACO2 fallan cerrados si se intenta iniciar modo hardware.

P3020 muestra cinco filas dentro de la misma interfaz. Debido a sus cinco
ejes, una petición solo de posición puede relajar la orientación. Una rotación
explícita que no puede alcanzarse se rechaza antes del controlador; esta
limitación también se muestra en el banner de la GUI.

### Simulación, seguridad y RViz

- Los adaptadores articular, cartesiano puntual, JOG y trayectoria cartesiana
  usan las articulaciones, frames y límites del perfil.
- `system_readiness` comprueba los nodos, controladores y endpoints de cada
  brazo.
- `profile_proximity_monitor`, `safety_supervisor` y `horizon_preview`
  comparten exactamente la misma geometría de cápsulas.
- RViz consume `robot_description`, `thesis/robot_capsules`,
  `thesis/horizon_volume` y `thesis/obstacle_visualization` dentro del
  namespace del perfil.
- La proyección incluye cápsulas actuales, volumen predicho, poses temporales,
  trazas de extremos y etiquetas de horizonte.

## Ejecución

Preparar cada terminal:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_historical_robot
```

Ejecutar solo una orden por vez:

```bash
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=jaco2
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=ur17g
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=tm25s
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=p3020
```

La GUI y RViz arrancan por defecto. Para automatización sin ventanas:

```bash
ros2 launch thesis_multi_robot historical_robot.launch.py \
  robot:=ur17g headless:=true start_gui:=false start_rviz:=false
```

## Alcance y límites

- Todo lo realizado y validado en esta ampliación es simulación.
- No se activó `hardware_output` ni se envió movimiento a un brazo físico.
- Los cuatro launch deben ejecutarse independientemente, no simultáneamente.
- P3020 no ofrece seguimiento cartesiano 6D arbitrario; conserva control
  articular y capacidad cartesiana de posición con rechazo seguro.
- La comprobación gráfica final confirmó `RobotModel/Status: Ok`, mallas 3D,
  materiales y proyección preventiva. Las capturas comparables de los cuatro
  brazos están en
  `evidence/codex_reports/2026-11-09_paridad_visual_modelos/`.

La evidencia reproducible está en
`evidence/codex_reports/2026-11-09_validacion_paridad_historica.md`.
