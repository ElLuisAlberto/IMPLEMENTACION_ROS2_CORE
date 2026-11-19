# Fuente CAD autocontenida: PincOpen instrumentado para JACO2

Este directorio conserva la fuente CAD necesaria para continuar la integración
del gripper de tesis sin depender de la papelera ni de un directorio externo al
workspace.

## Archivo conservado

| Archivo | SHA-256 | Estado |
| --- | --- | --- |
| `PINCOPEN_THESIS_CAD.zip` | `52ded9373e1f3aa165c7abc25fed9198ab4cd3377df2e9fb6cce97e05d13509b` | Copia íntegra, verificada con `unzip -t` |

El ZIP es una fuente inmutable: no debe editarse ni usarse directamente como
artefacto de ejecución de ROS 2. Los recursos que ROS sí puede cargar están en
`src/thesis_description/meshes/pincopen/`, con su manifiesto y licencia en el
mismo paquete.

## Contenido útil para la integración

- Ensamble JACO2: `04_ENSAMBLES/A20_SISTEMA_GRIPPER_JACO2.iam` y
  `02_FUENTES_INVENTOR/01_ENSAMBLES/SISTEMA_GRIPPER_INSTRUMENTADO_JACO2.iam`.
- Mecanismo interno: `04_ENSAMBLES/A10_GRIPPER_INSTRUMENTADO.iam` y piezas
  `.ipt` en `02_FUENTES_INVENTOR/`.
- Geometría intercambiable: STEP y STL de PincOpen en
  `01_PINCOPEN_ORIGINAL/cad/` y mallas visuales en
  `08_RECURSOS_SIMULACION/visual_meshes/original_mm/`.
- Trazabilidad, licencia y hashes: `00_PROCEDENCIA/` y
  `PACKAGE_MANIFEST_SHA256.csv`.

## Límite actual, deliberado

El propio paquete CAD declara como `TBD_*` la transformación
`jaco2_tool_flange -> gripper_mount`, los frames/pivotes, recorrido y sentido
del servo, tabla servo--apertura, masas, centros de masa, inercias y mallas de
colisión. Por ello el ZIP permite preservar la fuente y las mallas visuales,
pero **no autoriza aún un modelo cinemático o dinámico fiel**.

La siguiente extracción desde Inventor debe completar esos datos en archivos
versionados dentro de `src/thesis_description/` antes de habilitar un modo
`pincopen` de Gazebo. El modo normal del JACO2 debe permanecer en `legacy`
hasta entonces.
