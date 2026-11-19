# Integración de JACO2, UR17g-1300, TM25S y P3020 en simulación

Fecha: 2026-11-07. Entorno: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress
6.18.0, controlador `gz_ros2_control` ya instalado.

## Resultado y arquitectura

Se añadió `src/thesis_multi_robot` con cuatro perfiles que transforman las
descripciones oficiales a un backend común de simulación. Cada perfil tiene
prefijos de joints/links, robot_state_publisher, controller_manager y
arm_controller propios. Un único servidor Gazebo y un único puente de reloj
atienden la célula. El paquete selecciona fuentes mediante adaptadores;
ningún driver físico ni emulador de fabricante participa en la ejecución.

Los paquetes JACO2 existentes no se refactorizaron ni perdieron su ruta de
control validada. La nueva supervisión por perfil es experimental y utiliza
los contratos JointCommand, CommandDecision, Obstacle y ProximityStatus de
la tesis. No constituye equivalencia completa con la política JACO2 previa.

## Correcciones concretas

- UR: driver cambiado a Humble `223635513cba0c98e512758e4596509e9a3aed91`
  y descripción a Humble `78c5afd8bfee78a459c0265304340a85f1d44792`.
  Se evita compilar el simulador oficial que exige otro CMake y todavía no
  acepta este modelo en su launch. Se conserva descargado como referencia.
- TM25S: se usa `tm_description` Humble `9c087f6` y el control común,
  evitando las dependencias inválidas de `tm_gazebo` y los nombres globales
  de sus launches. Se conservan sus mallas, inercias y colisiones.
- P3020: se usa `dsr_description2` Humble `d276f9f` y el URDF estático que
  tiene 22 colisiones; se evita el Xacro que carece de ellas. Se respeta J4
  fijo y la cadena de cinco ejes. El control estándar hace innecesario
  Docker o el controlador virtual Doosan para esta simulación nominal.
- JACO2 en esta célula: los dedos se fijan a cero para que no queden joints
  pasivos moviéndose sin feedback y fuera del modelo de cápsulas. Esto se
  hace exclusivamente en el adaptador nuevo.
- Se añadió `COLCON_IGNORE` a `third_party/robots`: estar fuera de `src/`
  por sí solo no evita que colcon descubra los paquetes desde la raíz.
- Un script de build selecciona únicamente seis paquetes: las cuatro
  descripciones, interfaces y la integración.
- Se fijó trimesh 4.4.9 para compatibilidad con NumPy 1.21.5. Trimesh 5.2.0
  no importa con ese NumPy. No fue necesario instalar paquetes con sudo.

## Cinemática, geometría y supervisión

La FK se deriva del árbol URDF completo y la IK numérica respeta límites y
el número real de ejes. Las pruebas verifican que se conservan geometría,
inercias, ejes y límites de cada adaptador oficial. Las cápsulas contienen
cada geometría de colisión, aplicando escala y pose local; para P3020 se
comprobó cobertura de 237210 vértices en sus 22 mallas de colisión.

La supervisión comprueba antigüedad, orden de joints, límites, velocidades,
repetición de IDs y horizonte de movimiento. Considera un obstáculo
esférico externo y las cápsulas de los otros robots, con incertidumbre y
margen. La cantidad de muestras se adapta a una cota de desplazamiento de
2 cm entre intervalos, con presupuesto máximo de 400; si no alcanza,
rechaza la orden. Revalida antigüedad después de la evaluación.

Las comprobaciones entre robots se aceleraron mediante una cota inferior
de AABB para pares distantes y distancias vectorizadas para pares cercanos.
El benchmark de 21 muestras con 13×22 cápsulas cercanas pasó de 0.253 s
escalar a 0.0074 s vectorizado, con diferencia numérica cero.

Ante STOP, REDUCTION o datos caducados se solicita cancelación de la acción
activa; se verifica la respuesta y se reintenta hasta tres veces. El
controlador estándar entra en hold al aceptar la cancelación. Si no la
confirma, se publica CANCEL_FAILED sin afirmar una parada lograda.

## Validación

Build de los seis paquetes: aprobado.

Pruebas colcon del paquete nuevo: **74 aprobadas**, cero errores, cero
fallos. Incluyen geometría, FK/IK, perfiles, separación de namespaces,
colisión entre envolventes reales colocadas en la misma base, límites,
timestamps, muestreo y admisión. Un aviso de compatibilidad futura procede
de la librería COLLADA, sin afectar los resultados.

Regresión funcional por pytest, ejecutando cada paquete por separado:
`thesis_core` **173 aprobadas** y `thesis_simulation` **41 aprobadas**.
Se excluyeron los archivos de lint/copyright de esta regresión funcional.
La primera invocación conjunta encontró módulos de prueba con el mismo
nombre; ejecutar los paquetes por separado resolvió el problema de colección.

Prueba Gazebo final de los cuatro brazos en una partición aislada:

- Cuatro publicadores de estado frescos y cuatro supervisores ALLOW.
- Ocho controladores activos: broadcaster y arm_controller por robot.
- JACO2, UR y TM con seis ejes; P3020 con cinco.
- FK/TF coherentes dentro de 1 cm; error redondeado a seis decimales: 0 m.
- Cada J1 se movió +0.02 rad y volvió, con error inferior a 0.008 rad.
- Obstáculo insertado en movimiento: cancelación y hold en los cuatro,
  deriva observada aproximadamente 0.000001 rad.
- Rechazo por obstáculo en contacto y por fuente de obstáculo caducada;
  recuperación ALLOW al restaurar la fuente.
- Cuatro órdenes simultáneas de +0.01 rad: error inferior a 0.004 rad,
  sin STOP inesperado.
- Cierre limpio de todos los procesos de la prueba final.

Evidencias:

- `evidence/codex_reports/2026-11-07_multi_robot_cell_final/all_result.json`
- `evidence/codex_reports/2026-11-07_multi_robot_cell_final/all_launch.log`
- `evidence/codex_reports/2026-11-07_multi_robot_cell_final/unit_tests.xml`
- `evidence/codex_reports/2026-11-07_multi_robot_operator_cli_v2/p3020_result.json`:
  prueba adicional de lectura de descripción viva y envío de candidato por
  la herramienta del operador, aceptado en ALLOW. La primera prueba de esta
  herramienta detectó una API de Subscription ausente en Humble; se corrigió
  usando `Node.count_publishers` y la repetición pasó.
- Pruebas individuales y de la herramienta del operador en las carpetas
  con prefijo `2026-11-07_multi_robot_`.

Se preservaron también los intentos fallidos. Detectaron la resolución
duplicada de namespace del plugin Fortress y una carga de parámetros de
controladores que solo funcionaba para el último robot. La solución fue
usar robot_param_node relativo y pasar el YAML explícitamente al spawner
de cada arm_controller; la célula final confirmó las correcciones.

## Alcance pendiente

La célula nominal ya ejecuta los tres brazos nuevos junto al JACO2. No se
declara validación completa de seguridad ni emulación del firmware real.
Quedan fuera de las pruebas: autocolisión, frenado dinámico y sus
incertidumbres, retiming en REDUCTION, trayectorias cartesianas rectas,
integración de GUI JACO2, percepción RGB-D y ejecución MoveIt. La IK de una
pose produce un objetivo articular y no verifica una línea cartesiana.

Las instrucciones de ejecución y comandos supervisados están en
`src/thesis_multi_robot/README.md`. No se realizó commit ni se publicó nada.
