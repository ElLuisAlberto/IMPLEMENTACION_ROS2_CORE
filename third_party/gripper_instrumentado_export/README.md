# Entrega CAD para ROS 2 y Gazebo — Gripper instrumentado

Fecha de preparación: 2026-11-17.

## Alcance y trazabilidad

Fuente CAD comprobada:

- Proyecto Inventor: `02_FUENTES_INVENTOR/DISEÑOS_CAD_GENERAL.ipj` (fecha del archivo: 2026-10-22 10:40:40).
- Ensamble utilizado: `02_FUENTES_INVENTOR/01_ENSAMBLES/GRIPPER_ADAPTATIVO_INSTRUMENTADO.iam` (fecha: 2026-10-29 14:38:38).
- Copia de referencia comprobada: `04_ENSAMBLES/A10_GRIPPER_INSTRUMENTADO.iam`.

La unidad de longitud del documento es milímetro; el campo de revisión de iProperties está vacío (`Standard Revision=`), por lo que la fecha y el SHA-256 son la trazabilidad de revisión disponible.

Los dos ensamblajes tienen el mismo SHA-256:

`16ECA5D47D53BAC5F1CA3088C31B0F2C4F1482436E617D4BF39B8133CA304C32`.

Se abrió el ensamble fuente con 70 ocurrencias de primer nivel. No hubo referencias faltantes, ni componentes suprimidos u ocultos en la pose guardada. La lista íntegra de ocurrencias, visibilidad y archivos de origen está en `metadata/assembly_occurrences.csv`.

El alcance contiene la tapa/carcasa, acople del eje, leva, servo, bielas, eslabones, puntas removibles instrumentadas, soportes, insertos, pasadores y tornillería presentes en ese ensamble. No contiene el brazo JACO2 ni un adaptador JACO2; estos no se modificaron ni se han sustituido.

## Geometría entregada

- `assembly/gripper_complete.step`: ensamble completo, pose CAD guardada.
- `assembly/gripper_complete.stl`: malla del mismo conjunto y misma pose. El exportador se configuró con coordenadas en **mm** y resolución alta. Un STL no lleva unidades explícitas: al importarlo en ROS/Gazebo se debe interpretar como milímetros o aplicar escala `0.001` para metros.
- `rigid_bodies/*.step` y `rigid_bodies/*.stl`: 21 geometrías de documentos CAD referenciados directamente, para permitir crear los enlaces de simulación después de validar la topología de juntas. No se han unido piezas en cuerpos rígidos sin comprobación.
- `captures/front.png`, `side.png`, `top.png` e `isometric.png`: vistas del mismo estado exportado.

El documento Inventor usa milímetros como unidad de longitud y kilogramos como unidad de masa. Como control de escala, la envolvente del ensamble en la pose exportada es:

| Eje CAD | Mínimo (mm) | Máximo (mm) | Extensión (mm) |
| --- | ---: | ---: | ---: |
| X | -53.680 | 76.144 | 129.824 |
| Y | -62.598 | 66.840 | 129.437 |
| Z | -13.063 | 46.437 | 59.500 |

Los valores proceden de `physical_properties/gripper_complete_cad_properties.csv`.

## Origen, transformaciones y montaje

`metadata/occurrence_transforms.csv` proporciona para cada una de las 70 ocurrencias una matriz de rotación 3×3 y traslación en **metros**, relativas al origen nativo del ensamble Inventor. Inventor trabaja internamente en cm; las traslaciones se convirtieron a m en la tabla.

Ese origen se conserva como `inventor_assembly_origin` en `kinematics/frames.yaml`. Aún no se ha declarado como `gripper_base` ni como `gripper_mount`: la superficie de montaje requiere una selección geométrica explícita en Inventor antes de fijar el frame de interfaz para UR17g, P3020 o CS625.

Convención provisional de exportación, no una validación mecánica: las puntas y carcasas con Y positiva se listan como candidata izquierda y las de Y negativa como candidata derecha en `metadata/rigid_body_grouping.csv`. Confirmar esa convención al elegir el frame de montaje.

## Estado de cinemática

No se entregan ejes, límites, TCP ni 15 muestras numéricas como valores validados. El diagnóstico de 149 restricciones devolvió 142 al día y 7 con estado **driver lost**. Las restricciones afectadas incluyen la leva/servo y bielas/eslabones; por ello no es seguro arrastrar el ensamble ni deducir ángulos, apertura, interferencias o agrupación rígida.

Los archivos de `kinematics/` conservan las plantillas y registran expresamente los elementos pendientes. `servo_to_opening.csv` incluye 15 filas de adquisición bloqueadas para llenar tras reparar las restricciones. Los límites de PincOpen que aparecen en `limites_mecanicos.yaml` son referencia histórica upstream y **no** deben cargarse en URDF, ros2_control ni Gazebo como límites locales.

Para completar el modelo cinemático hace falta reparar y comprobar las siete restricciones, definir `gripper_mount` y `gripper_base`, identificar el eje/sentido del servo y pivotes, y después tomar las muestras de apertura durante apertura y cierre.

## Propiedades físicas

`physical_properties/cad_occurrence_mass_properties.csv` contiene propiedades de masa CAD por ocurrencia. El ensamble completo devuelve 0.167021876 kg, centro de masa `[-0.010156, 0.000289, 0.007553] m` y tensor completo en `physical_properties/gripper_complete_cad_properties.csv` / `propiedades_masa.csv`.

Esos datos son propiedades CAD, no parámetros finales de Gazebo: las apariencias/materiales activos son etiquetas genéricas que deben validarse, y aún no se ha asignado con certeza cada tornillo, sensor y componente del servo a un cuerpo rígido. `cad_appearance_materials.csv` documenta esas etiquetas; `materiales.csv` conserva los parámetros pendientes de contacto/fricción.

## Uso recomendado en ROS 2 / Gazebo

El paquete autocontenido listo para copiar a un workspace ROS 2 está en `ros2_ws/src/gripper_instrumentado_description/`. Contiene una única malla del ensamble completo, URDF/Xacro, modelo SDF de Gazebo Sim, mundo y dos lanzadores. Su README explica los comandos `colcon build`, RViz 2 y `gz sim`.

La versión incluida carga el gripper en la pose CAD de manera estática: es la forma correcta de preservar la representación 3D mientras se repara la cinemática. Para colisión dinámica y `ros2_control`, esperar a reparar las restricciones y convertir las agrupaciones candidatas de `metadata/rigid_body_grouping.csv` en enlaces validados. No montar todavía sobre UR17g, P3020 ni CS625: este paquete describe únicamente el gripper independiente y deja el adaptador fuera de alcance.

## Archivos de control

- `metadata/inventor_constraints_health.csv`: salud de las 149 restricciones.
- `metadata/pending_measurements.md`: lista concreta de mediciones aún necesarias.
- `metadata/cad_appearance_materials.csv`: apariencias y materiales CAD registrados.
- `metadata/assembly_occurrences.csv`: inventario, estado y referencias.
- `metadata/occurrence_transforms.csv`: poses relativas al origen de Inventor.
