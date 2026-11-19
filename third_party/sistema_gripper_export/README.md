# Sistema gripper instrumentado para ROS 2 y Gazebo

Esta entrega se generó desde `SISTEMA_GRIPPER_INSTRUMENTADO_JACO2.iam`, usando el proyecto Inventor `DISEÑOS_CAD_GENERAL.ipj` el 2026-11-17.

## Alcance exacto

El modelo contiene los 15 componentes de primer nivel del ensamble fuente, todos visibles y sin suprimir, incluidos:

- `MODULO_ELECTRONICA_PERCEPCION_RGBD:1`;
- `GRIPPER_ADAPTATIVO_INSTRUMENTADO:1`;
- los cuatro FSR `FSR_DF9-40`;
- tornillos M3 y los demás elementos de fijación de nivel superior.

No se incluyó un brazo JACO2. El nombre histórico del ensamble conserva `JACO2`, pero este paquete representa el módulo de herramienta independiente que se ve en Inventor.

## Paquete que se debe usar

Copiar o descomprimir `ros2_ws/` en un entorno Linux con ROS 2. El paquete único es:

`ros2_ws/src/sistema_gripper_instrumentado_description`

Contiene el modelo SDF de Gazebo Sim, URDF/Xacro, mundo, lanzadores y ambas mallas. Para ejecutarlo:

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sistema_gripper_instrumentado_description gazebo_visual.launch.py
```

Gazebo usa el GLB visual generado directamente por Inventor: contiene la jerarquía del ensamblaje, colores, 113 materiales y una textura integrada. El STL de la misma pose se reserva para colisión y para la vista de respaldo de RViz 2.

## Archivos principales

- `assembly/sistema_gripper_instrumentado.step`: CAD ensamblado completo.
- `assembly/sistema_gripper_instrumentado.stl`: malla completa para colisión, en mm.
- `ros2_ws/src/sistema_gripper_instrumentado_description/models/sistema_gripper_instrumentado/meshes/sistema_gripper_instrumentado.glb`: visual con apariencias CAD, en m.

El modelo se entrega estático como módulo completo. La visual es fiel a la pose CAD; la cinemática de apertura/cierre del subensamble permanece pendiente hasta reparar las restricciones perdidas de `GRIPPER_ADAPTATIVO_INSTRUMENTADO.iam`. Tampoco se activó una cámara simulada: antes de simular sensores RGB-D se debe definir y validar su frame óptico, orientación, parámetros intrínsecos y tópico ROS 2.

## Escala y referencias

- Documento Inventor: milímetros.
- STL: importar como mm o escalar `0.001` a m.
- GLB: exportador Inventor ya aplica escala de `0.01` de coordenadas internas cm a m; no escalar otra vez.
- Origen: origen nativo de `SISTEMA_GRIPPER_INSTRUMENTADO_JACO2.iam`.
- Referencias faltantes: 0.

La caja de envolvente CAD de la pose exportada mide aproximadamente 121.272 × 198.636 × 129.437 mm (X × Y × Z). La masa CAD agregada es 0.539520 kg; no se usa para dinámica hasta validar materiales, cableado y agrupaciones rígidas.
