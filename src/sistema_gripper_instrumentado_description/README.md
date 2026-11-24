# sistema_gripper_instrumentado_description

Este es el único paquete ROS 2 que se debe compilar para visualizar el sistema completo de electrónica/percepción, gripper, FSR y fijaciones. No representa el brazo JACO2.

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sistema_gripper_instrumentado_description gazebo_visual.launch.py
```

Gazebo Sim usa `sistema_gripper_instrumentado.glb`, exportado por Inventor y con las apariencias del ensamble. Para RViz y los brazos simulados se usa `sistema_gripper_instrumentado_colores.dae`, convertido desde ese GLB a metros con los colores de sus materiales; la textura integrada no se convierte. El STL se conserva para la geometría de colisión y de proximidad. La conversión se puede repetir con `python3 tools/assets/export_gripper_collada.py` desde la raíz del proyecto, con NumPy y pycollada ya disponibles.

El módulo se mantiene estático: no se han definido juntas ni sensores simulados para no inventar límites, frames ópticos ni parámetros de percepción. El siguiente paso, cuando se requiera, es separar los cuerpos del mecanismo y validar la cinemática del gripper.
