# Incorporación de fuentes para gemelos digitales

Fecha: 2026-11-07

Este documento registra la descarga inicial. La integración posterior,
las correcciones de ramas y las pruebas ejecutadas están en
`2026-11-07_integracion_multi_robot_fortress.md`.

## Resultado

Se incorporaron como submódulos Git cinco repositorios oficiales que cubren
los tres robots seleccionados. No se modificaron los paquetes JACO2 ni se
conectaron estas fuentes al sistema de control.

| Fabricante y modelo | Repositorio | Commit fijado |
| --- | --- | --- |
| Universal Robots UR17g-1300 | `Universal_Robots_ROS2_Driver` | `e55b899066caffbceda80662d63b61c80f56719a` |
| Universal Robots UR17g-1300 | `Universal_Robots_ROS2_Description` | `0eb1a861f7772fceb38510d90fa2f98ec668070c` |
| Universal Robots UR17g-1300 | `Universal_Robots_ROS2_GZ_Simulation` | `f232204099fdfb4d503b792c9a1ef2d896b121e2` |
| OMRON/Techman TM25S | `tm2_ros2` | `9c087f600e06f7bbaf96ec0f3844fca47b843262` |
| Doosan P3020 | `doosan-robot2` | `d276f9f9712118a18bad1d9ee1d7d4eab10a4ea8` |

## Verificación estática

- UR17g-1300: se localizaron su configuración visual y las referencias del
  modelo en la descripción y el driver oficiales.
- TM25S: se localizaron URDF/Xacro, mallas, launch de Gazebo y configuración
  MoveIt específica.
- P3020: se localizaron URDF/Xacro, configuración `ros2_control`, Gazebo y
  paquete MoveIt específico.
- Los cinco repositorios contienen su archivo de licencia.
- Los submódulos permanecen fuera de `src/`; por tanto no entran en la
  compilación actual del workspace.

## Riesgo de compatibilidad

TM25S y P3020 ofrecen ramas Humble. El soporte reciente de UR17g-1300 está en
las ramas activas `main`/`rolling`, no en la rama Humble histórica. Antes de
crear el perfil UR17g debe evaluarse si sus Xacro y plugins pueden portarse a
Humble/Gazebo Fortress sin incorporar dependencias incompatibles.

No se ejecutó compilación porque las fuentes aún están deliberadamente
aisladas y no existe un paquete de integración que seleccione únicamente los
componentes necesarios.
