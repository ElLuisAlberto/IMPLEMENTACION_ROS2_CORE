# Fuentes oficiales de robots colaborativos

Este directorio mantiene, como submódulos Git, las fuentes oficiales de los
dos robots activos de la célula. `COLCON_IGNORE` evita su
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
- Las dos descripciones activas se compilaron en Humble. UR quedó fijado a sus
  backports oficiales Humble: descripción `78c5afd`, driver `2236355`.
- La célula usa sus propios wrappers y controladores Gazebo estándar; no
  compila los paquetes Gazebo/MoveIt/driver incompatibles de los fabricantes.
- `thesis_multi_robot` ofrece supervisión experimental por perfil; los
  launches JACO2 originales y su ruta física siguen independientes.
- Cada submódulo conserva las licencias originales. Las mallas recientes UR
  incluyen condiciones adicionales de Graphical Documentation.

Ver [la guía de la célula](../../src/thesis_multi_robot/README.md) para
compilación, ejecución, comandos supervisados y alcance de las pruebas.

## Candidato Elite CS625

La descripción oficial del CS625 se descargó localmente en
`elite/ros2_description`, rama `humble`, commit
`bb8ca600424c5b47e3e41d93d92e320506305f98`:
<https://github.com/EliteRobots/Elite_Robots_CS_ROS2_Description>.
Incluye las mallas, cinemática, límites y propiedades del modelo y declara
licencia Apache-2.0. Se compiló por selección explícita y el URDF expandido
pasó `check_urdf`. Aún no está integrado en la célula ni probado con torque.

Esta copia es un clon Git independiente, **no un submódulo registrado en el
repositorio padre**: no se recupera con `git submodule update`. Para reproducir
la descarga, clonar la URL anterior en esa ruta y seleccionar el commit
indicado. Antes de versionarla como submódulo hay que incorporar su gitlink y
`.gitmodules` sin sobrescribir los cambios ya preparados en el índice.
