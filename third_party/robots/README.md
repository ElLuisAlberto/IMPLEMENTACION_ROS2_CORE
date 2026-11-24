# Fuentes oficiales de robots colaborativos

Este directorio mantiene, como submódulos Git, las fuentes oficiales de los
fabricantes representados en la célula. `COLCON_IGNORE` evita su
descubrimiento accidental desde la raíz. El script
`tools/simulation/build_multi_robot.sh` selecciona explícitamente las
descripciones que utiliza `src/thesis_multi_robot`.

## Organización

| Robot objetivo | Fuente | Rama registrada | Contenido comprobado |
| --- | --- | --- | --- |
| Universal Robots UR17g-1300 | `universal_robots/ros2_driver` | `humble` | Referencia; driver no utilizado por la célula |
| Universal Robots UR17g-1300 | `universal_robots/ros2_description` | `humble` | URDF/Xacro y parámetros del modelo, compilados |
| Universal Robots UR17g-1300 | `universal_robots/ros2_gz_simulation` | `ros2` | Referencia; incompatible con Humble y no compilada |
| Doosan P3020 | `doosan/doosan_robot2` | `humble` | URDF/Xacro, `ros2_control`, Gazebo y MoveIt para P3020 |
| Elite Robots CS625 | `elite/ros2_description` | `humble` | Descripción oficial, mallas, cinemática y límites |

Los commits exactos quedan fijados por el repositorio padre. Para recuperar
las fuentes después de clonar esta tesis:

```bash
git submodule update --init --recursive
```

También se puede clonar todo desde el inicio con:

```bash
git clone --recurse-submodules <URL_DEL_REPOSITORIO>
```

## Compatibilidad y alcance actual

- La tesis usa ROS 2 Humble y Gazebo Fortress.
- Las descripciones activas se compilaron en Humble. UR quedó fijado a sus
  backports oficiales Humble: descripción `78c5afd`, driver `2236355`.
- La célula usa sus propios wrappers y controladores Gazebo estándar; no
  compila los paquetes Gazebo/MoveIt/driver incompatibles de los fabricantes.
- `thesis_multi_robot` ofrece supervisión experimental por perfil; los
  launches JACO2 originales y su ruta física siguen independientes.
- Cada submódulo conserva las licencias originales. Las mallas recientes UR
  incluyen condiciones adicionales de Graphical Documentation.

Ver [la guía de la célula](../../src/thesis_multi_robot/README.md) para
compilación, ejecución, comandos supervisados y alcance de las pruebas.

## Elite CS625

La descripción oficial del CS625 está registrada como submódulo en
`elite/ros2_description`, rama `humble`, commit
`bb8ca600424c5b47e3e41d93d92e320506305f98`:
<https://github.com/EliteRobots/Elite_Robots_CS_ROS2_Description>.
Incluye las mallas, cinemática, límites y propiedades del modelo y declara
licencia Apache-2.0. El perfil está integrado en la simulación y comparte la
supervisión experimental; no se han validado esfuerzos ni control físico.
