# gripper_instrumentado_description

Nota de integración: la entrega original se conserva íntegra en
`third_party/gripper_instrumentado_export/` con su manifiesto SHA-256. El
Xacro de esta copia declara la propiedad `mesh_scale` a partir del argumento
correspondiente para permitir su expansión con ROS 2 Humble. La malla conserva
el hash del archivo original. El lanzador de Gazebo suministrado requiere
`gz sim`; su compatibilidad con Gazebo Fortress no ha sido comprobada.

Paquete ROS 2 autocontenido para cargar en RViz 2 o Gazebo Sim la geometría
completa del ensamble `GRIPPER_ADAPTATIVO_INSTRUMENTADO.iam` en la pose CAD
exportada. El único archivo de malla que necesita este paquete es:

`models/gripper_instrumentado/meshes/gripper_complete.stl`.

La malla procede del ensamble completo y se escala de mm a m mediante
`0.001 0.001 0.001`. No hay archivos externos ni geometría del JACO2.

## Instalación

Copiar esta carpeta `gripper_instrumentado_description` dentro de `src` de un
workspace ROS 2 en Linux, por ejemplo:

```bash
mkdir -p ~/gripper_ros2_ws/src
cp -a gripper_instrumentado_description ~/gripper_ros2_ws/src/
cd ~/gripper_ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Cargar el ensamble

Para visualizarlo en RViz 2:

```bash
ros2 launch gripper_instrumentado_description view.launch.py
```

Para cargar el mismo ensamble estático en Gazebo Sim (Harmonic o una versión
compatible con el comando `gz sim`):

```bash
ros2 launch gripper_instrumentado_description gazebo_visual.launch.py
```

## Limitación deliberada

El modelo es **estático** y contiene un único enlace `gripper_base`. Por tanto
su forma y pose sí son las del CAD, pero no abre ni cierra todavía. El ensamble
Inventor registra siete restricciones `DriverLost` que afectan leva, bielas y
eslabones; no se ha fabricado una cinemática ni límites ficticios.

Tras reparar esas restricciones, el siguiente trabajo consiste en sustituir el
URDF estático por enlaces rígidos agrupados y juntas validadas. Las mallas,
transformaciones y plantillas necesarias para ello están en los directorios
superiores de la entrega (`rigid_bodies/`, `metadata/` y `kinematics/`).
