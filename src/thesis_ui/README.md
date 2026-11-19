# Interfaz de escritorio JACO2

La interfaz `joint_gui` abre ahora un espacio de trabajo maximizado con colores
marfil, arena y terracota. Conserva los controles y contratos ROS de la interfaz
monobrazo anterior. La barra de disponibilidad y estado permanece visible.

## Abrir el control existente

Desde el workspace preparado:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run thesis_ui joint_gui
```

Los launches existentes que arrancan `joint_gui` utilizan el nuevo diseño
automáticamente. Si el pipeline ya abrió la GUI, utiliza esa ventana.
Los parámetros de simulación, disponibilidad y operación física siguen siendo
los del launch correspondiente.

La navegación contiene Resumen, Articular / Manual, Objetivo cartesiano,
Trayectorias, Diagnóstico e Historial. Los modos cartesianos puntuales y de
trayectoria siguen apareciendo solamente en simulación. Al abandonar la página
Articular / Manual se desactiva el JOG mediante los callbacks existentes;
regresar a ella no reanuda el movimiento.

Diagnóstico e Historial tienen el botón **Abrir en ventana**. Cada ventana
reutiliza el panel original y vuelve a colocarlo en su página al cerrarse.
Cerrar una auxiliar no termina ROS ni el control principal.

F11 alterna pantalla completa; Esc vuelve al modo maximizado.
La distribución se adapta a 1366×768 y 1920×1080. En pantallas estrechas los
paneles se apilan y cada página tiene desplazamiento vertical propio.

Para usar la presentación anterior con el mismo backend:

```bash
ros2 run thesis_ui joint_gui --ros-args -p ui_layout:=legacy
```

Al reabrir una GUI física se deben conservar también los parámetros del comando
físico documentado en el README general. Este selector solo cambia el diseño;
no selecciona un brazo, un backend ni una autorización física.

## Vista de los cuatro brazos del escenario híbrido

Con el escenario `hybrid_three_sim.launch.py` ya iniciado, en otra terminal
preparada y con el mismo dominio ROS:

```bash
ros2 run thesis_ui multiarm_gui
```

Este tablero es de observación. `arm_01` es la fuente sintética sin actuador;
`arm_02`, `arm_03` y `arm_04` son los brazos de Gazebo. Presenta sus articulaciones,
la disponibilidad híbrida y la evaluación global recibida. No crea publicadores
de órdenes ni clientes de control. La antigüedad indicada es de recepción local
y no sustituye las validaciones del coordinador.

El tablero no añade control multi-brazo a la GUI monobrazo. Esa capacidad no
existía en la interfaz anterior y necesita una integración separada de la ruta
preventiva local y global. Las rutas existentes del escenario no se modifican.

## Obstáculo de la prueba individual

La simulación completa sigue por defecto la esfera visible de Gazebo:

```bash
ros2 launch thesis_simulation thesis_system.launch.py \
  obstacle_source_mode:=gazebo simulation_output_enabled:=true
```

Mover esa esfera actualiza la distancia evaluada. Para una estimación virtual o
externa se selecciona `obstacle_source_mode:=topic`; el demo fijo se habilita con
`use_demo_obstacle:=true`. En este modo la estimación se ve en RViz y no se crea
una segunda esfera de Gazebo ajena a ella. El banner identifica la fuente.
La GUI muestra falta de geometría como dato no disponible, conservando STOP.

Diagnóstico y validación de esta corrección:
`docs/codex_reports/2026-11-13_diagnostico_falsa_proximidad_simulacion.md` y
`evidence/codex_reports/2026-11-13_validacion_proximidad_simulada.md`.

## Comprobación

```bash
QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  -p no:cacheprovider \
  src/thesis_ui/test/test_gui_state.py \
  src/thesis_ui/test/test_workspace_layout.py \
  src/thesis_ui/test/test_multiarm_observer.py
```

Los resultados y límites de la campaña están en
`evidence/codex_reports/2026-11-12_validacion_redisenio_interfaz.md`.
