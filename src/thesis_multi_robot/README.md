# Célula nominal JACO2 + UR17g + P3020 + CS625

**Actualización:** La célula incluye también CS625 y monta el CAD fijo del
gripper en los tres brazos comerciales. Su envolvente se incorpora a la
supervisión geométrica experimental. El montaje es estimado, los dedos son
fijos y no hay masa ni inercia de la herramienta: no usar esta representación
para validar esfuerzos bajo carga ni seguridad física. Se solicita la interfaz
de esfuerzo articular para observación, pero aún no hay política de seguridad
basada en esfuerzo ni validación de sus valores. Las descripciones históricas
de esta guía que mencionan solo tres perfiles preceden a esta actualización.

Este paquete ejecuta los tres brazos activos en un único mundo Gazebo Fortress,
con controladores, joints y TF propios. Se construyen las descripciones
oficiales; la capa de simulación y la supervisión por perfil son de la tesis.

## Preparación

Desde la raíz del repositorio:

```bash
git submodule update --init --recursive
python3 -m pip install --user --no-deps trimesh==4.4.9 pycollada==0.9.3
bash tools/simulation/build_multi_robot.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_multi_robot
```

Se utilizan NumPy, SciPy y PyYAML del entorno Humble. La versión de trimesh
está fijada para conservar compatibilidad con NumPy 1.21.5. No actualizar
NumPy de ROS para satisfacer una versión más reciente de trimesh.

El script de construcción instala también `thesis_core`, `thesis_ui` y
`thesis_simulation`: tras construir esta célula, el flujo histórico de JACO2
sigue disponible desde el mismo `install/setup.bash`.

## Aislamiento respecto al JACO2 histórico

No iniciar `thesis_system.launch.py` y `cell.launch.py` en el mismo dominio
ROS ni en la misma partición de Gazebo. Ambos publican `/clock` y el marco
raíz `world`; aunque los controladores de la célula tengan namespace, compartir
esos recursos contaminaría las dos simulaciones.

Usar, por ejemplo, esta separación:

```bash
# Célula multi-robot
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_multi_robot

# En otra terminal, solo si se necesita el JACO2 histórico en paralelo
export ROS_DOMAIN_ID=87
export IGN_PARTITION=thesis_jaco2_legacy
ros2 launch thesis_simulation thesis_system.launch.py \
  start_gui:=false start_rviz:=false headless:=true
```

La segunda orden se ejecuta desde una terminal con el mismo workspace fuente,
pero nunca con las variables de entorno de la célula multi-robot.

## Ejecución

Los tres brazos activos, sin interfaz gráfica:

```bash
ros2 launch thesis_multi_robot cell.launch.py
```

Para abrir Gazebo, añadir `headless:=false`. Para un brazo aislado, usar
`robot:=ur17g`, `robot:=p3020` o `robot:=jaco2`.

## Ruta individual con la interfaz histórica completa

La ruta recomendada para comparar la misma capa de seguridad entre brazos es
`historical_robot.launch.py`. Inicia un solo brazo y reutiliza el mismo
ejecutable `thesis_ui/joint_control_gui.py`, el supervisor preventivo, los
adaptadores cartesianos, la predicción temporal y los marcadores de horizonte
que usa el JACO2 histórico. La tabla articular, los límites, la cinemática, las
cápsulas y los endpoints se cargan desde el perfil del brazo.

El caso `robot:=jaco2` no carga el perfil genérico: deriva directamente a
`thesis_simulation/thesis_system.launch.py`. Así conserva los nombres globales,
la cinemática y las seis cápsulas históricas. Los otros dos casos se encapsulan
en su namespace y mantienen el mismo contrato de mensajes con remappings. Si
se omite `robot:=...`, el wrapper selecciona JACO2; el launch interno rechaza
`jaco2 + historical_stack` para que esta ruta no pueda evitarse por accidente.

Desde una terminal nueva:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_historical_robot
```

Ejecutar un brazo por vez:

```bash
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=jaco2
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=ur17g
ros2 launch thesis_multi_robot historical_robot.launch.py robot:=p3020
```

La GUI y RViz arrancan por defecto. Para CI o diagnóstico sin ventanas:

```bash
ros2 launch thesis_multi_robot historical_robot.launch.py \
  robot:=ur17g headless:=true start_gui:=false start_rviz:=false
```

El P3020 muestra los mismos controles en una tabla de cinco ejes. Una petición
solo XYZ puede relajar la orientación porque el brazo no tiene seis grados de
libertad; una rotación explícita imposible se rechaza y no llega al controlador.

## Interfaz nominal de la célula multi-robot

`start_gui:=true` abre la misma interfaz para JACO2, UR17g y P3020.
La tabla articular se adapta automáticamente a 6/6/5 ejes. La ventana
permite copiar el estado actual, enviar un objetivo articular, aplicar un
incremento pequeño y resolver un objetivo cartesiano puntual mediante la
cinemática del URDF. Para P3020 se recomienda activar **Solo posición**, pues
sus cinco ejes no pueden satisfacer una orientación arbitraria.

La interfaz publica únicamente en
`/<robot>/thesis/candidate_command`. El supervisor del perfil conserva la
admisión, los límites, la comprobación geométrica y la cancelación automática
por `STOP`. Los botones permanecen bloqueados hasta recibir estado y
proximidad frescos; un objetivo se marca `REACHED` solo después de la
confirmación terminal del controlador.

En cada terminal nueva se prepara el entorno así:

```bash
cd /home/luis/Escritorio/TESIS_JACO2/IMPLEMENTACION_ROS2_CORE
source /opt/ros/humble/setup.bash
source install/setup.bash
```

UR17g aislado:

```bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_ur17g_gui
ros2 launch thesis_multi_robot cell.launch.py \
  robot:=ur17g headless:=false start_gui:=true
```

P3020 aislado:

```bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_p3020_gui
ros2 launch thesis_multi_robot cell.launch.py \
  robot:=p3020 headless:=false start_gui:=true
```

Los bloques anteriores se ejecutan uno por vez, tal como se validaron. Para
ver los tres perfiles activos en una sola ventana y elegir el brazo activo:

```bash
export ROS_DOMAIN_ID=88
export IGN_PARTITION=thesis_all_gui
ros2 launch thesis_multi_robot cell.launch.py \
  robot:=all headless:=false start_gui:=true
```

También puede abrirse el JACO2 de esta célula con `robot:=jaco2`. La
simulación histórica del JACO2 y su GUI avanzada permanecen disponibles por
`thesis_simulation thesis_system.launch.py`; no se modificaron ni se deben
mezclar con esta célula en el mismo dominio o partición.

La célula completa monta las bases en `world`:

| Perfil | Base XYZ (m) | Ejes activos | Controlador |
| --- | --- | --- | --- |
| jaco2 | -4, 0, 0 | 6 | `/jaco2/arm_controller` |
| ur17g | 0, 0, 0 | 6 | `/ur17g/arm_controller` |
| p3020 | 0, 4, 0 | 5 (1,2,3,5,6) | `/p3020/arm_controller` |

Un perfil aislado se monta en el origen. JACO2 conserva su geometría, pero
sus dedos se fijan a cero en esta célula para no dejar articulaciones sin
control que discrepen de las cápsulas. El launch JACO2 original no cambia.
Los archivos URDF, YAML y `cell.json` se generan en un directorio temporal que
se elimina al apagar el launch.

## Comandos supervisados

En otra terminal con el mismo entorno y dominio:

```bash
python3 tools/simulation/command_robot.py --robot ur17g \
  --joint-index 1 --delta 0.02 --duration 2
```

También admite `--xyz X Y Z` (metros absolutos en `world`), `--rpy R P Y`
(radianes) o `--position-only`. La IK numérica respeta los límites y los
cinco ejes del P3020. Una orden XYZ produce un objetivo articular: la
trayectoria resultante interpola articulaciones, no una línea cartesiana.

La herramienta lee la descripción del simulador y publica únicamente en
`/<robot>/thesis/candidate_command`. Devuelve la decisión del supervisor;
`accepted=true` confirma admisión, no movimiento completado. El estado se
observa en `/<robot>/joint_states`.

## Supervisión experimental

Cada perfil usa FK desde su URDF y cápsulas que contienen todas sus
geometrías de colisión oficiales. P3020 usa el URDF estático con sus 22
colisiones; no el Xacro de fabricante que carece de ellas. No se necesita
Docker, firmware virtual, drivers de hardware ni MoveIt para esta ruta.

La supervisión comprueba límites, velocidad articular máxima 0.15 rad/s,
timestamps, antigüedad de 0.5 s, IDs repetidos, horizonte de trayectoria,
esfera externa y cápsulas de los otros robots. El muestreo adapta el número
de intervalos para limitar el desplazamiento estimado entre muestras a
2 cm; rechaza órdenes que requieren más de 400 intervalos. Usa margen
geométrico de 5 cm, además de incertidumbre del obstáculo y cobertura de
mallas de 5 mm. La separación lejana entre robots puede reportarse como una
cota inferior conservadora de AABB.

`ALLOW` y `WARNING` admiten; `REDUCTION` rechaza hasta disponer de retiming;
`STOP` rechaza y cancela la acción activa. Se comprueba la respuesta de
cancelación y se reintenta hasta tres veces. Si el controlador no confirma,
se publica `CANCEL_FAILED`; no se afirma que se haya alcanzado hold.

La esfera demo se publica lejos de los brazos. Usar `demo_obstacle:=false`
para reemplazarla por una fuente propia `thesis_interfaces/msg/Obstacle` en
`/<robot>/thesis/obstacle`, frame `world`. Cada supervisor exige estados
frescos de todos los robots presentes en su manifiesto.

Esta ruta es una base experimental para comparar perfiles. La interfaz
unificada cubre el subconjunto nominal común de control articular, incrementos,
IK puntual y feedback de supervisión. Todavía no
equivale a toda la política JACO2 existente: faltan validación de
autocolisión, retiming de REDUCTION, incertidumbre y frenado dinámicos,
trayectorias cartesianas rectas y comparación completa con el supervisor
original. El JOG continuo, las trayectorias cartesianas largas cancelables,
la percepción RGB-D y MoveIt siguen siendo funciones exclusivas o pendientes
de la ruta histórica; no se afirma paridad avanzada entre supervisores.
Los ejemplos no establecen autorización para movimiento físico.

## Validación reproducible

```bash
colcon test --base-paths src/thesis_multi_robot --packages-select thesis_multi_robot
colcon test-result --test-result-base build/thesis_multi_robot --verbose
python3 tools/simulation/validate_cell.py --robot all \
  --output evidence/codex_reports/FECHA_NUEVA_prueba_celula --domain 88
python3 tools/simulation/validate_multi_robot_gui.py --robot all \
  --output evidence/codex_reports/FECHA_NUEVA_prueba_gui --domain 89
```

La prueba integrada crea su propia partición Gazebo y dominio ROS, inicia
la célula, comprueba controladores y FK/TF, mueve cada J1 +0.02 rad y lo
devuelve, inserta un obstáculo durante otra trayectoria y mide hold,
rechaza movimientos en contacto y con datos caducados, y detiene los
procesos iniciados. La carpeta de salida debe ser nueva para preservar
evidencias anteriores.

El validador de la interfaz comprueba reloj simulado, callbacks, grados de
libertad, controladores activos, movimiento y retorno de cada perfil sin
cruce de comandos, además de iniciar el entry point Qt en modo offscreen. Se
puede repetir con `--robot ur17g`, `--robot p3020` o
`--robot jaco2` para probar un lanzamiento aislado.
# Vista CAD provisional del gripper

Con los paquetes compilados, `ros2 launch thesis_multi_robot
provisional_gripper_view.launch.py robot:=ur17g` abre una vista en RViz;
también se admite `p3020` o `cs625`. El montaje es una estimación visual
provisional (eje CAD +Y alineado al eje +Z de la brida, offset 63 mm) y una
caja cubre la malla estática con margen de 1 mm. No se
ha validado el montaje ni el recorrido. Esta vista no inicia Gazebo, control
de movimiento ni supervisión; los deslizadores solo cambian la vista en RViz.
No usar esta geometría como evidencia de seguridad física.

Para previsualizar el mismo conjunto completo en Gazebo Fortress, sin ningún
controlador ni supervisor y con el robot estático:

```bash
ros2 launch thesis_multi_robot provisional_system_gazebo.launch.py robot:=ur17g
```

Se puede sustituir `ur17g` por `p3020` o `cs625`. La vista usa la malla STL
completa a escala milímetros a metros, incluyendo electrónica y cámara; el
STL es monocromo. La cámara no es un sensor simulado y la apertura está fija.
La pose de montaje es una estimación visual, no un adaptador validado.

El CS625 también está disponible en la célula supervisada mediante
`robot:=cs625`. Las vistas provisionales siguen siendo estáticas; el esfuerzo
y el montaje de herramienta aún requieren validación.
