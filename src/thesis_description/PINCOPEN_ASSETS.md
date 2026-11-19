# Activos PincOpen para el gripper de tesis

Este paquete contiene una copia sin modificar de las catorce mallas visuales
publicadas por Pollen Robotics para PincOpen. Se importaron desde
`PINCOPEN_THESIS_CAD.zip`, paquete `0.1.0-cad-preparation`, en
`meshes/pincopen/original_mm/`.

Procedencia: Pollen Robotics, [PincOpen](https://github.com/pollen-robotics/PincOpen),
revisión `1b5a3dc83cb4eae042d535bda3765b9e12a9b8a7`. Las mallas y cualquier
derivado se distribuyen bajo CC BY-SA 4.0; el texto completo está en
`PINCOPEN_LICENSE`.

## Estado de integración

Los STL son únicamente activos visuales de referencia. Sus coordenadas están
en milímetros y se deben declarar con escala uniforme `0.001 0.001 0.001` al
usarlos desde URDF/Xacro. La macro reutilizable
`urdf/pincopen_visual_assets.xacro` aplica esa escala, pero no se carga por
defecto en ningún robot.

No se ha activado un modo `pincopen` en `j2n6s300.xacro`: las transformaciones
`JACO2 -> gripper_mount`, los frames de los cuerpos, límites, masas, inercias,
colisiones simplificadas y la curva servo--apertura del ZIP siguen pendientes
de medición o validación. El modo `interface` existente conserva los frames
reservados y es el único modo admisible para esta fase.

En particular, estas mallas no son colisiones y no deben alimentar una decisión
de seguridad ni una simulación dinámica. Una integración posterior debe:

1. validar el ensamble `A20_SISTEMA_GRIPPER_JACO2.iam` y medir los frames;
2. exportar mallas de colisión conservadoras por separado;
3. definir el perfil versionado brazo + gripper por estado observado; y
4. habilitar actuación solo después de validar límites, cinemática y masa.

Ejecute `python3 tools/validation/validate_pincopen_visual_assets.py` desde la
raíz de `IMPLEMENTACION_ROS2_CORE` para comprobar los nombres, tamaños y hashes
de los activos importados contra el manifiesto del ZIP.
