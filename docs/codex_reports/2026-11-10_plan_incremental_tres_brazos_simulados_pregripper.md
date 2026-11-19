# Plan incremental: tres JACO2 simulados preparados para un futuro gripper

Fecha: 2026-11-10, zona del usuario Asia/Manila. Estado: planeamiento; no implementación.

## 1. Objetivo y alcance acordado

Preparar `arm_02`, `arm_03` y `arm_04` como tres JACO2 independientes en una única simulación, con una interfaz estable para incorporar después el gripper de tesis. El modelo CAD, las dimensiones, articulaciones, masas y límites de ese gripper aún no están disponibles.

El sistema final previsto combina esos tres simulados con el JACO2 físico (`arm_01`). Su gripper averiado no se accionará ni será reemplazado por el gripper de tesis durante este avance. Si sigue montado, su volumen debe estar representado y comprobado antes de utilizar su geometría en una decisión de seguridad física.

El trabajo previo al CAD se completará enteramente offline: tres robots y tres controller managers en Gazebo, más una fuente lógica de estado sintético para `arm_01`. Esta cuarta identidad permite evaluar los seis pares de robots sin crear un cuarto actuador ni cargar el bridge, SDK o bibliotecas Kinova. La observación del JACO2 físico queda como una fase futura independiente.

Este documento es el resultado del análisis del agente principal y de tres agentes especializados en arquitectura, seguridad y pruebas. Incluye su discusión cruzada y las correcciones de consenso. No autoriza movimiento físico ni contiene una afirmación de validación del gripper definitivo.

## 2. Punto de partida comprobado

Referencia del repositorio: rama `main`, HEAD `7cb7522`. El workspace contiene cambios previos que no están todos en ese commit; HEAD por sí solo no describe el estado auditado. Hay seis archivos rastreados modificados y módulos/configuraciones/pruebas nuevos sin rastrear, además de material bibliográfico. Al implementar se debe registrar el diff y el inventario de esos archivos sin reemplazarlos ni descartarlos.

La limpieza autorizada anterior retiró `build/`, `install/` y `log/`. ROS Humble está disponible en `/opt/ros/humble`; la reconstrucción del subconjunto necesario y sus dependencias será el primer paso operativo de una implementación posterior.

| Elemento existente | Evidencia del estado actual | Consecuencia para el plan |
| --- | --- | --- |
| Contrato de cuatro instancias | `src/thesis_simulation/config/multiarm_simulation.yaml` y `thesis_simulation/multiarm_config.py` | Declara cuatro simulados; no crea entidades ni distingue fuentes físicas/sintéticas. Conservarlo y añadir un escenario nuevo. |
| Descripción del JACO2 | `src/thesis_description/urdf/j2n6s300.xacro` | La macro acepta `prefix`, pero incorpora siempre los dedos heredados; `link_6` usa la malla `hand_3finger`. |
| Simulación y control | `jaco_gazebo.launch.py`, `jaco_gazebo.ros2_control.xacro`, `jaco_gazebo_controllers.yaml` | Entidad, joints, topics y controller manager siguen siendo monobrazo. |
| Nombres y adaptadores | `thesis_core/joint_model.py`, `simulation_command_adapter.py`, adaptadores cartesianos | Esperan nombres `j2n6s300_*`; un namespace por sí solo no resuelve joints, frames ni topics absolutos. |
| Base geométrica del supervisor | `jaco_kinematics.py`, `horizon_clearance.py` y comparación TF/modelo en `safety_supervisor_node.py` | La FK actual parte de identidad y asume la base en el origen; desplazar/rotar las bases exige adaptar también las predicciones locales y el interlock TF/FK. |
| Readiness de simulación | `system_readiness_node.py` | Cuenta basenames de nodos y espera un controlador; debe conocer instancias y nombres completamente calificados. |
| Contrato del gripper | `src/thesis_description/urdf/thesis_gripper_interface.xacro` | Solo crea frames de montaje, base y TCP; no se incluye en los modelos activos. |
| Geometría versionada | `geometry_profile.py`, `geometry_snapshot_projection.py` | Hay perfiles, estados y proyección matemática a `world`; falta su adaptación runtime a mensajes y TF. |
| Coordinación global | `multiarm_coordination.py` | Evalúa cuatro ocupaciones y seis pares actuales con cápsulas/esferas. No hay nodo ROS ni protección predictiva interbrazo. |
| Mensajes | `GripperState.msg`, `GeometrySnapshot.msg` | Están declarados, pero no hay cadena runtime estado de herramienta → ocupación → decisión global. |
| Proximidad local | `proximity_monitor_node.py`, `jaco_capsules.yaml` | El monitor admite explícitamente el perfil `jaco2_legacy`; no basta cambiar el nombre del perfil para añadir geometría. |
| Supervisor y bridge físico | `safety_supervisor_node.py`, `jaco_hardware_node.cpp` | Usan rutas globales monobrazo. No reciben una restricción global multiarm integrada. |
| Observación física | `src/thesis_hardware/launch/jaco_read_only_state.launch.py` | Existe un launch de estado, fuerza salida física deshabilitada y usa tiempo real; no es un coordinador híbrido. |

En la auditoría anterior de esta misma sesión se ejecutaron y pasaron 37 pruebas puras de cinco suites:

- `test_geometry_profile.py`.
- `test_geometry_snapshot_projection.py`.
- `test_multiarm_coordination.py`.
- `test_multiarm_config.py`.
- `test_multiarm_safety_contract.py`.

Se usó `python3 -m pytest`, con escritura de bytecode y caché desactivadas. El ejecutable `pytest` no estaba en PATH, pero el módulo Python sí estaba disponible. Las 37 pruebas prueban contratos matemáticos y configuración; no prueban lanzamiento de Gazebo, mensajes ROS generados, comportamiento de controladores ni detención integrada. No se repitieron al redactar este documento porque no se modificó código.

## 3. Discusión entre agentes y decisiones resultantes

Los agentes intercambiaron propuestas, objeciones y réplicas directamente; el agente principal contrastó sus resultados con los archivos actuales.

| Punto discutido | Resolución y motivo |
| --- | --- |
| Cuatro robots simulados frente a tres simulados y una fuente lógica | Se corrigió expresamente una propuesta de cuatro entidades. La topología inicial tendrá exactamente tres entidades y tres controller managers: `arm_02..04`. `arm_01` será `synthetic_state`, sin entidad Gazebo ni actuador. |
| Prefijos en URDF frente a prefijos adicionales de robot_state_publisher | Se aplicará un prefijo real una sola vez a links y joints en Xacro. Los adaptadores recibirán el mapa de nombres por instancia; no se añadirá un segundo `frame_prefix` de robot_state_publisher. |
| Cambiar el escenario actual de cuatro simulados | Se añadirán configuración y launch nuevos; los contratos y defaults monobrazo actuales se conservarán como regresión. |
| Añadir ya un gripper ficticio funcional | Se prepararán la interfaz y las pruebas de estados/envolventes. Las geometrías de ensayo serán fixtures sintéticos con medidas definidas por el caso de prueba, sin atribuirles las medidas, actuadores o dinámica del diseño pendiente. |
| Seguridad global a partir del monitor de un obstáculo | Se mantendrá la semántica local de ProximityStatus. La coordinación multiarm tendrá su propio estado y se compondrá con las restricciones locales mediante una única autoridad de salida por actuador. |
| Movimiento cartesiano al principio | Las primeras pruebas serán articulares. La cinemática cartesiana con TCP definitivo depende del montaje del futuro gripper y se validará después. |
| Mezclar directamente reloj físico y /clock | Queda rechazado. La fase offline utilizará un solo reloj simulado; la observación física tendrá su propio dominio temporal y no habilitará decisiones hasta validar una adaptación explícita. |
| Considerar terminado el trabajo solo cuando se conecte el brazo físico | Se acordó que P0–P6 son suficientes para terminar la preparación offline. La observación física es futura y opcional para ese cierre. |
| Llamar predictiva a la evaluación actual | Se distinguirá la coordinación instantánea de la futura coordinación predictiva. Tendrán pruebas y conclusiones separadas. |

## 4. Arquitectura objetivo antes de recibir el CAD

```text
Un proceso/mundo Gazebo y un publicador de /clock
  arm_02: JACO2 + RSP + ros2_control + adaptador simulado
  arm_03: JACO2 + RSP + ros2_control + adaptador simulado
  arm_04: JACO2 + RSP + ros2_control + adaptador simulado

arm_01: fuente synthetic_state + modelo cinemático/TF sin actuador

Estados + perfiles + TF de las cuatro identidades
  → ocupaciones completas en world en un tiempo de evaluación común
  → coordinador de los seis pares
  → restricción global con vigencia y motivo
  → composición con las restricciones locales de cada simulado
  → una salida autorizada por cada controlador arm_02..04

Telemetría: decisiones, tiempos, fuentes, perfiles, fallos y control aplicado
```

La representación de `arm_01` puede visualizarse como modelo fantasma en RViz mediante estado/TF sintético, sin convertirla en un cuarto robot de Gazebo. Un replay offline debe declarar una transformación de tiempos validada hacia la sesión simulada y conservar sus tiempos originales como evidencia; no se reutilizarán sellos antiguos ni se cambiará su fecha al recibirlos sin esa trazabilidad. El primer incremento usará un generador determinista en el mismo `/clock` para evitar esa dependencia.

### 4.1. Identidades, topics y nombres

Las rutas siguientes son el contrato propuesto; se fijarán y probarán en P1 antes de generar launch y controladores. Los archivos propuestos aún no existen.

| Recurso | arm_01 inicial | Ejemplo arm_02; análogo en arm_03/04 |
| --- | --- | --- |
| Rol/fuente | `synthetic_state` | `gazebo` |
| Namespace | `/arm_01` | `/arm_02` |
| Entidad Gazebo | Ninguna | `arm_02_jaco` |
| Prefijo real URDF/TF | `arm_01_jaco` | `arm_02_jaco` |
| Estado articular | `/arm_01/joint_states` | `/arm_02/joint_states` |
| Descripción | `/arm_01/robot_description` | `/arm_02/robot_description` |
| Controller manager | Ninguno | `/arm_02/controller_manager` |
| Acción de brazo | Ninguna | `/arm_02/arm_controller/follow_joint_trajectory` |
| Frames de herramienta reservados | Según el modelo sintético de prueba | `arm_02_jaco_tool_mount`, `arm_02_jaco_gripper_base`, `arm_02_jaco_gripper_tcp` |
| Salida permitida | `none` | `simulation` |

Las descripciones, comandos, ejecución, control runtime, readiness y telemetría deben tener rutas específicas por robot. Los topics globales actuales `/thesis/*`, `/joint_states` y las acciones absolutas requieren remapeo explícito o parámetros; aplicar `namespace` al Node no cambia los nombres absolutos.

`/tf` y `/tf_static` pueden compartir transporte si cada child frame tiene nombre único y autoridad única. `world` es el único marco global del experimento: cada base debe tener una transformación conocida y una sola autoridad. La pose de spawn Gazebo y la transformación TF de la base deben representar la misma pose sin aplicar dos veces el desplazamiento. No se crearán cuatro marcos llamados `world` con padres/transformaciones distintas.

La FK actual parte de una matriz identidad y sus consumidores la interpretan como geometría world. Para bases trasladadas/rotadas se requiere `world_from_base` por instancia en la geometría/predicción local, o una conversión rigurosamente equivalente de todos sus datos al mismo marco. Se preservará identidad como default legado. La comparación TF/modelo del supervisor y los testigos publicados deben aplicar la misma transformación una vez. Parametrizar joints sin corregir esta suposición produciría distancias falsas o un interlock permanente; no se pospondrá este requisito a P7.

`world_from_base` pertenecerá a la configuración/identidad de la instancia y será coherente con el sello de estado, TF y obstáculos. Se aplicará a toda la ocupación y trayectoria: cápsulas, herramienta, TCP y predicciones. Si se expresan datos world en coordenadas de base, también se rotarán correctamente las velocidades/vectores, sin aplicarles la traslación de puntos. Una transformación ausente o incoherente retirará autorización; el default identidad del legado nunca será fallback silencioso de una instancia desplazada. Se probará la equivalencia de una misma escena expresada en world o base y de FK transformada frente a TF al sello exacto.

El default de `JOINT_NAMES` seguirá describiendo el monobrazo legado. Los adaptadores multiinstancia recibirán listas explícitas de seis nombres y validarán orden, unicidad y correspondencia con su URDF/controlador. La matemática que opera con vectores de seis articulaciones no necesita conocer namespaces. La instancia no modificará una constante global para cambiar el comportamiento de las demás.

El escenario nuevo debe validar exactamente una fuente de estado por identidad, exactamente tres backends Gazebo y ninguna salida física. Cambiar de generador sintético a fuente física será un cambio de modo con reinicio/invalidez de la sesión, nunca dos publicadores simultáneos para `arm_01`.

### 4.2. Mano heredada, montaje y TCP

El JACO2 actual ya incorpora `kinova_3fingers`; además, su `link_6` usa una malla de mano y el efector final tiene un offset legado de 0.160 m. Quitar solo los dedos no demuestra que exista una brida geométricamente correcta para el gripper nuevo.

Se preparará un selector de herramienta para los simulados y se conservará el modelo legado como default. El selector deberá distinguir al menos el modelo heredado, la interfaz sin herramienta nueva y la fixture de ensayo. La interfaz sin geometría solo reserva frames: no acredita que un gripper real ocupe volumen cero. Ningún modo debe superponer el gripper nuevo a los dedos heredados accidentalmente.

Se fijará un solo punto de montaje, con transformaciones configurables `mount_xyz/rpy` y `tcp_xyz/rpy`. Hasta tener el CAD, sus valores serán referencias explícitas de integración/ensayo. La transformación al TCP de prueba se verificará con FK y no se interpretará como calibración física. Cuando llegue el CAD se decidirá con medidas si se usa/reemplaza la malla de mano heredada y dónde está la brida real. No se sumará automáticamente el offset de 0.160 m al nuevo montaje.

### 4.3. Geometría, mensajes y tiempo

Una ocupación debe representar el robot completo: brazo más herramienta seleccionada y, si corresponde, carga conocida. Reemplazar las seis cápsulas del brazo por una esfera del gripper dejaría al resto del manipulador fuera de la evaluación.

`GeometrySnapshot.msg` contiene descriptores locales y referencias a frames; no es una lista de posiciones world ya calculadas. P1 debe decidir y documentar una frontera única: publicar el descriptor local con su sello, resolver todos sus TF en el adaptador/coordinador y entregar `RobotOccupancySnapshot` al núcleo. Si se introduce un mensaje de ocupación world, tendrá un nombre y contrato distintos. No se asignará el campo `world` a primitivas que aún estén expresadas en frames locales.

La serialización debe validar arrays paralelos de igual longitud, tipos de primitivas admitidos, radios positivos, coordenadas finitas, nombres únicos y límites de tamaño. El perfil, versión, hash, estado, origen y revisión de sesión deberán referirse al mismo conjunto de primitivas; no se aceptará una mezcla de revisiones de brazo y herramienta.

El adaptador resolverá todos los TF al tiempo de adquisición/evaluación acordado. Si falta uno, se descarta la ocupación completa; no se recurre silenciosamente a `Time()`/último TF. Cada conjunto debe identificar las fuentes y tiempos originales usados. La evaluación simultánea podrá elegir un tiempo común dentro de la ventana con datos válidos de las cuatro fuentes y requerirá interpolación disponible; extrapolar o volver a fechar estados viejos no es una forma de sincronización.

Se exigirán por separado: TTL por fuente, recepción reciente según reloj monotónico, límite de skew entre adquisiciones, secuencias/épocas de sesión y coherencia del reloj ROS. El núcleo actual solo verifica edades individuales; estos controles faltantes deberán añadirse en la frontera runtime y cubrirse en pruebas.

En offline, todos los estados/TF geométricos usarán `/clock`. Los watchdogs de disponibilidad deben seguir detectando fallos con reloj monotónico aunque Gazebo esté pausado. Inicio sin `/clock`, tiempo cero no inicializado, pausa prolongada, salto hacia atrás/adelante y reinicio invalidarán readiness/autorizaciones hasta restablecer un conjunto nuevo coherente. El coordinador podrá existir publicando STOP/NOT_READY mientras espera datos; no otorgará ALLOW por haber arrancado el proceso.

Los perfiles sintéticos tendrán identificadores y procedencia de ensayo, diferentes de los perfiles reales futuros. Un hash demuestra identidad de un artefacto, no que sus dimensiones cubran una pieza física. El manifiesto debe abarcar geometría de seguridad, estados, mapa de frames y parámetros relevantes, no solo una malla visual.

El estado `desconocido` solo podrá resolver a una envolvente máxima vacía de carga cuando el contrato y las envolventes lo permitan explícitamente. Si existe posibilidad de carga no identificada, estado inválido o falta un perfil requerido, la decisión es STOP. La política ya presente en `GeometryProfile.envelope_for` se preservará y ampliará con pruebas runtime. No se reutilizará ese fallback para ocultar la geometría desconocida del gripper físico averiado.

Las transiciones de herramienta deben tener una envolvente que cubra el recorrido intermedio o invalidar la autorización durante el cambio. Una orden de cerrar no permite declarar el volumen cerrado antes de confirmarlo por estado; el contrato distingue estado solicitado y observado. Los fixtures pueden probar esta regla sin inventar el mecanismo real.

### 4.4. Autoridad de control

El estado global multiarm tendrá su propio contrato de identidad, vigencia, par/primitivas limitantes y razón de fallo. No se fingirá que tres manipuladores son un único obstáculo esférico en `ProximityStatus`.

En la primera integración se usará una restricción global binaria ALLOW/STOP para las tres salidas simuladas. Las políticas locales existentes podrán seguir reduciendo velocidad. La autorización efectiva requiere todas las restricciones vigentes y válidas; la escala efectiva es el mínimo permitido. Un STOP local/global o una restricción requerida ausente/vencida impone escala cero. ALLOW global no elimina un STOP local ni rearma el sistema.

Una sola autoridad por actuador emitirá el control final: no habrá dos publicadores que alternen órdenes ALLOW y STOP al mismo controlador. La restricción global debe comprobarse tanto al aceptar candidatos como durante trayectorias ya aceptadas, JOG y colas pendientes. Detener incluye cancelar/prevenir replanificaciones pendientes y producir el hold/deceleración definido por el adaptador, verificando el resultado en feedback; publicar un estado STOP no demuestra que el robot se haya detenido.

Las decisiones incluirán secuencia/época y caducidad. No se utilizará un ALLOW latched de una sesión anterior como autorización vigente. El QoS se fijará y probará por tipo de mensaje: descripción/manifiesto estático y estado dinámico no tienen la misma política de persistencia. Colas acotadas y control de edad deben impedir que un consumidor atrasado siga aplicando permisos antiguos.

La pérdida del coordinador debe producir STOP por un watchdog independiente del proceso que falló. Para el primer hito, el STOP global afectará los tres controladores simulados. El bridge físico queda fuera de esta ruta; una identidad sintética no puede recibir una orden física.

## 5. Incrementos de implementación y criterios de salida

### P0 — Base reproducible y límites de la campaña

Dependencias: ninguna. Riesgo principal: confundir cambios previos o mensajes generados antiguos con la nueva implementación.

Trabajo:

1. Registrar HEAD, diff, archivos no rastreados relevantes y hashes de configuración. Preservar todas las modificaciones existentes; no hacer limpieza adicional ni commits automáticos.
2. Identificar el cierre de dependencias necesario tras retirar build/install; compilar ese subconjunto antes de importar mensajes ROS. Verificar imports de `GripperState`, `GeometrySnapshot` y nuevos campos de `ProximityStatus`.
3. Repetir las 37 pruebas puras como referencia y añadir al baseline aplicable las pruebas de mensajes generados, geometría/proximidad, monobrazo y adaptador de simulación.
4. Crear una ficha de campaña con modo OFFLINE, exactamente tres salidas de simulación y ausencia de bridge/SDK físico. Fijar directorio de evidencias y versión de configuración.

Salida G0: baseline reproducible, dependencias identificadas y pruebas de referencia registradas. Un fallo del baseline se diagnostica antes de construir las fases siguientes; no se atribuye automáticamente al cambio nuevo.

Para cerrar G0 deben existir: manifiesto versionado con HEAD/diff/inventario y entorno; lista exacta de paquetes reconstruidos y resultado de build; comprobación de imports/campos de mensajes generados; y salida con recuentos de las suites seleccionadas, sin fallos. Una dependencia ausente o una prueba requerida omitida deja el gate pendiente, con motivo registrado.

Artefactos: manifiesto de baseline y resultado de pruebas en `evidence/codex_reports/`. No se requiere CAD ni dispositivo físico.

### P1 — Contratos de escenario, nombres, herramienta, tiempo y decisión

Dependencia: G0. Riesgo principal: generar después componentes incompatibles entre sí.

Trabajo:

1. Añadir un escenario nuevo con cuatro identidades lógicas y roles: `arm_01=synthetic_state/output_none`, `arm_02..04=gazebo/output_simulation`. Rechazar configuraciones incompletas, IDs extra, fuentes dobles, entidades/controladores duplicados y cualquier selección de salida física en este launch.
2. Fijar mapa de joints/frames/topics/controladores, `world_from_base` por instancia y política de autoridades TF; validar nombres de entidades, namespaces y prefijos como conceptos relacionados pero distintos.
3. Especificar selector de herramienta y procedencia de mount/TCP. Definir cómo la ocupación compone brazo, fixture y carga sin perder primitivas.
4. Especificar mensajes runtime: descriptor geométrico, estado de herramienta, decisión global, fuentes/sello/TTL/época y estado de readiness. Reutilizar mensajes existentes cuando respeten esa semántica; introducir tipos nuevos solo cuando la frontera lo requiera.
5. Fijar política temporal offline, máximos de edad/skew configurables, timeout monotónico y reinicio. Los valores iniciales serán parámetros de ensayo, no márgenes físicos validados.
6. Definir la composición de restricciones y la autoridad única por actuador antes de conectar publicadores a comandos.

Salida G1: pruebas puras del escenario y serialización; matrices de nombres completas; casos inválidos rechazados; contrato escrito que P2 y P4 puedan implementar sin decisiones contradictorias. Preservar el YAML actual y las pruebas que esperan cuatro simulados.

Artefactos obligatorios G1: esquema/validador y escenario efectivos; tabla de las cuatro identidades con roles/rutas/joints/frames/transformaciones; contrato de estado/descriptor/decisión con tiempos y fallos; manifiesto de perfiles/fixtures; y resultados de los casos de rechazo por ID/fuente duplicada, backend prohibido, mapa incompleto, tipo/array inválido y parámetro temporal ausente/incoherente. Se validan contratos y conversión pura en este gate; la entrega/recepción ROS integrada se demuestra en G4.

Archivos previstos: nuevos `hybrid_scenario.yaml` y módulo de validación/mapa por instancia en `thesis_simulation`; cambios compatibles en `thesis_interfaces` si el contrato exige tipos/campos nuevos. Nombres de archivos nuevos sujetos al contrato de P1.

### P2 — Una instancia simulada parametrizada y conexión de herramienta preparada

Dependencia: G1. Riesgo principal: multiplicar tres veces un monobrazo con nombres o geometría incorrectos.

Trabajo:

1. Parametrizar el wrapper Xacro de Gazebo y ros2_control para una instancia de ensayo `arm_02`: prefijo, mapa de seis joints, configuración de controlador y base pose coherente con TF/spawn. Parametrizar también `robot_param_node` y `controller_manager_name` del plugin `gz_ros2_control`, de forma que cada plugin use el RSP y `robot_description` de su propio namespace mediante rutas verificadas, no los literales globales del monobrazo.
2. Extraer o introducir una unidad de lanzamiento por robot que no inicie un Gazebo adicional, un segundo reloj ni un pipeline monobrazo global.
3. Adaptar los consumidores iniciales de joint states y control articular para recibir nombres y rutas por instancia. Mantener los nombres actuales como defaults del monobrazo.
4. Integrar el selector de herramienta y los frames de montaje/base/TCP. Conservar la variante heredada por defecto; declarar claramente los modos sin herramienta nueva y de fixture. No inventar articulaciones ni accionamientos del gripper definitivo.
5. Probar que las seis articulaciones controladas se corresponden exactamente con URDF, ros2_control, YAML, comandos y feedback. Probar variante legada y variante prefijada, así como FK/TF para una base trasladada y rotada con `world_from_base` explícito. Preparar el contexto geométrico que necesitarán las restricciones locales de P5.

Salida G2: una instancia prefijada controlable en joint-space, sin colisiones de nombres; configuración monobrazo de referencia equivalente; un solo montaje/TCP de prueba sin doble offset ni superposición de manos. La expansión Xacro por sí sola no satisface este gate: se debe comprobar la interfaz de control activa.

Archivos afectados: `thesis_description/urdf/j2n6s300.xacro` y/o wrapper compatible, `thesis_gripper_interface.xacro`, `thesis_simulation/urdf/jaco_gazebo.ros2_control.xacro`, plantillas de controladores, adaptador de comandos y pruebas de mapas. No es necesario cambiar aún la cinemática cartesiana ni la ruta de comandos física.

### P3 — Tres entidades en un Gazebo y cuarta fuente lógica

Dependencia: G2. Riesgo principal: cruces de comandos/estado y falsas condiciones de disponibilidad.

Trabajo:

1. Crear el launch nuevo que inicia un solo servidor/mundo Gazebo, tres entidades `arm_02..04`, tres RSP y tres controller managers. Usar el único publicador de `/clock` del mundo Gazebo, a través de un único bridge si es necesario; impedir un segundo publicador/bridge que compita como autoridad temporal.
2. Iniciar el generador determinista `arm_01=synthetic_state` con estados y TF propios en el mismo reloj. No crear entidad ni controlador para esa identidad.
3. Iniciar/controlar cada robot con dependencias de servicios/eventos y timeouts. Evitar depender solo de sleeps fijos para asumir que Gazebo o el controlador están listos.
4. Crear readiness por instancia y del conjunto. Verificar FQN de nodos, controladores realmente activos, feedback válido y fresco, TF completo y reloj; no contar basenames como si las réplicas fueran duplicados.
5. Ejecutar movimientos articulares pequeños de prueba en un robot simulado a la vez y después en los tres. Comprobar tanto destinatario correcto como ausencia de cambios inesperados en los otros.
6. Mantener perfiles de colocación distintos: uno separado para arranque/aislamiento y fixtures de ocupación controlados para ensayos de proximidad. Las bases separadas actuales no garantizan que se ejerciten umbrales; no reducir sin justificación el mínimo de separación para forzar un caso.

Salida G3: exactamente tres entidades, tres CMs, un reloj y cuatro fuentes lógicas válidas; comandos aislados; salida de simulación deshabilitada mientras falten prerrequisitos; cero proceso/SDK/endpoint de control físico iniciado por el launch offline. Una caída parcial retira readiness.

Archivos previstos: launch multiinstancia nuevo, generador `synthetic_state`, configuración de poses/mundo y readiness multiinstancia; pruebas de lanzamiento headless. La GUI es opcional y no será condición de readiness de una campaña headless.

### P4 — Perfiles compuestos y coordinador ROS en observación

Dependencias: G1 y G3 para integración; el código puro de esta fase puede desarrollarse en paralelo a P2/P3 después de G1. Riesgo principal: ocupación parcial, estados de gripper ignorados o sellos incoherentes.

Trabajo:

1. Registrar perfiles del brazo legado y perfiles de fixtures sintéticos, con composición explícita brazo+herramienta. Para los tres simulados sin fixture, declarar exactamente la variante geométrica usada; no declarar un gripper instalado que no exista.
2. Añadir un publicador de estado de herramienta de ensayo, sin driver físico. Ejercitar estados de envolvente abierta/cerrada/nominal, carga conocida y desconocida; sus nombres son etiquetas de test y no una reproducción de un mecanismo inexistente.
3. Implementar conversión/validación de mensajes y resolución completa de TF al sello acordado. Adjuntar procedencia, mapa de frames, estado y revisión.
4. Implementar coordinador ROS sobre el núcleo actual con conjunto canónico obligatorio, perfiles esperados obligatorios y los controles adicionales de TTL, edad de recepción, skew, dominio temporal y época. La interfaz runtime debe rechazar el conjunto antes de invocar el núcleo si falta una época/sesión común, el perfil esperado de cualquier identidad, la política de skew/adquisición común o un TF disponible al sello exacto. No permitir que `expected_geometry=None` eluda la validación en el runtime.
5. Publicar estado global, clearance y testigos limitantes de los seis pares, junto con motivos de invalidez/readiness. Inicialmente observar sin conectar órdenes.
6. Comparar resultados runtime con fixtures geométricos de respuesta conocida y con los resultados del núcleo puro. Cubrir todas las combinaciones cápsula–cápsula, cápsula–esfera y esfera–esfera.

Salida G4: decisiones observables y reproducibles para cuatro identidades; ninguna evaluación parcial se presenta como válida; fallos de fuente, frame, estado, hash, versión, reloj o sincronización dan STOP/NOT_READY con causa identificable. Los escenarios demuestran que también se evalúan primitivas de herramienta/carga y no solo el brazo.

Archivos afectados: `geometry_profile.py`, `geometry_snapshot_projection.py`, adaptación de mensajes/perfiles, nodo coordinador nuevo y entry_points en `thesis_core/setup.py`; `multiarm_safety_contract.py` ampliado mediante contrato nuevo compatible, más interfaces/pruebas necesarias.

### P5 — Restricción global aplicada a los tres controladores simulados

Dependencia: G4. Riesgo principal: tener una decisión correcta que no interrumpe una ejecución en curso.

Trabajo:

1. Componer restricción global y local por instancia con una autoridad final de comandos/control. La política global inicial es binaria; las reducciones locales no se pierden. Adaptar el monitor local y el supervisor a sus nombres y a `world_from_base`, preservando las comprobaciones canónicas pertinentes; el interlock TF/FK y el predictor local deben comparar magnitudes en el mismo marco para bases trasladadas/rotadas.
2. Exigir permiso vigente para admitir candidatos y para mantener ejecuciones/JOG. Cubrir colas, replans, resultados tardíos de acciones y cambios de generación.
3. Aplicar STOP a los tres adaptadores simulados y comprobar cancelación, hold/deceleración y feedback de parada. Añadir watchdog de la decisión global en el consumidor, con reloj monotónico y fallo cerrado.
4. Probar que un ALLOW tardío, retenido o de época anterior no libera un STOP. Definir recuperación por conjunto nuevo estable y sin reanudar automáticamente una orden terminal/cancelada.
5. Ensayar apagado del coordinador, caída de un controlador, ausencia de TF y pérdida de estado; registrar detección y propagación a las tres salidas.

Salida G5: el STOP global bloquea nuevas órdenes e interrumpe ejecuciones de `arm_02..04`; se verifica en feedback dentro del presupuesto experimental declarado. La prueba incluye trayectorias ya aceptadas y JOG, no solo candidatos nuevos. `arm_01` no tiene endpoint de actuación y no recibe comandos.

Archivos afectados: integración compatible en `safety_supervisor_node.py` y/o compositor por instancia, `simulation_command_adapter.py`, readiness y contratos de control. No reutilizar un ProximityStatus esférico como transporte del estado multiarm.

### P6 — Campaña reproducible y cierre de preparación sin CAD

Dependencia: G5. La instrumentación y los casos se diseñarán desde P1 y se incorporarán durante P3–P5; esta fase consolida la evidencia, no la añade al final de manera improvisada.

Trabajo:

1. Ejecutar la matriz de la sección 6, guardar la configuración efectiva y comprobar la regresión monobrazo aplicable.
2. Medir coste del coordinador, edades/skew, jitter, pérdida de muestras, deadlines y latencia de propagación STOP. Distinguir estado publicado de control aplicado y de velocidad observada.
3. Hacer campaña de escalado por número de primitivas y fuentes activas, manteniendo escenario, semillas y configuración de control. Para comparaciones con menos brazos se usarán benchmarks específicos del núcleo; no se desactivará silenciosamente la exigencia de cuatro identidades del runtime de producción.
4. Preparar el manifiesto que deberá completar el CAD: variante de mano/brida, montaje/TCP, visual/collision, articulaciones/actuación, límites, masas/inercia y perfiles de estados/carga.
5. Publicar una ficha de estado con tres resultados diferenciados: listo para incorporar un modelo de herramienta; seguridad instantánea integrada en simulación; protección predictiva interbrazo pendiente si P7 no se ha completado.

Salida G6: los tres simulados están aislados y controlados, las interfaces de herramienta se instancian por brazo, las fixtures prueban la cadena de estados/geometría, STOP funciona en simulación y la evidencia es reproducible. Se declara lo que queda pendiente del CAD y del sistema físico. Este es el criterio de terminación del avance previo solicitado.

Archivos previstos: harness nuevo en `tools/validation/`, instrumentación/lector de métricas multiinstancia en `thesis_telemetry` y pruebas correspondientes, sin alterar informes históricos.

### P7 — Extensión predictiva entre brazos, como gate separado

Dependencias: G4 para diseñar/codificar el núcleo y G5/G6 para validar su efecto integrado. Puede hacerse antes del CAD utilizando cápsulas/fixtures, pero no se confundirá con G6.

El predictor actual de un brazo frente a un obstáculo no equivale a la predicción de dos robots móviles. Se necesita un contrato de trayectorias/velocidades/referencias aceptadas por robot, con tiempos compatibles, incertidumbre, horizonte y estado real como origen. Una orden candidata no se tratará como movimiento ya ejecutado.

Trabajo:

1. Evaluar los seis pares a tiempos futuros comunes, con geometría de herramienta/carga correspondiente a cada estado. Conservar por separado clearance actual, mínimo futuro y tiempo al evento.
2. Diseñar conservadurismo entre muestras: barrido, subdivisión o cotas de movimiento verificables. Muestrear un horizonte sin considerar el intervalo entre muestras puede omitir cruces rápidos.
3. Incorporar frescura de predicciones y límites de velocidad. Si se estudia reducción de velocidad, reevaluar la trayectoria temporal resultante; dos robots móviles no se resuelven multiplicando sin más una escala monobrazo.
4. Probar cruces con clearance actual seguro y mínimo futuro inseguro, movimientos opuestos, frenado/retardo y herramientas cambiando de envolvente. Separar autocollisión de colisión interbrazo; el núcleo actual no prueba autocollisión del robot/herramienta.
5. Medir latencia total y declarar un presupuesto de reacción. Sin medidas de frenado y geometría real no se deducirán distancias físicas de parada ni garantías para hardware.

Salida G7: evidencia predictiva independiente y documentada; solo entonces se podrá atribuir al experimento protección predictiva multiarm. ADVERTENCIA de alcance técnico: completar G6 sin G7 no habilita esa conclusión.

## 6. Matriz de aceptación y rendimiento

Todos los casos de esta sección son pruebas propuestas. No se ejecutaron durante este planeamiento.

| ID | Caso | Resultado exigido | Gate |
| --- | --- | --- | --- |
| T01 | Expansión URDF/Xacro legada y tres variantes prefijadas | Joints/links internos coherentes; defaults legados equivalentes; nombres globales únicos para las tres instancias. | G2 |
| T02 | Correspondencia URDF–ros2_control–YAML–adaptador | Mismos seis joints y orden validado; ningún controlador apunta a otro robot. | G2/G3 |
| T03 | Montaje/TCP, selector y base trasladada/rotada | Un solo montaje y offset; sin doble mano; fixture no se identifica como CAD real; FK/TF y clearance se conservan bajo cambios rígidos coherentes de marco. | G2/G5 |
| T04 | Grafo y lanzamiento headless | Tres entidades/CM, cuatro fuentes lógicas, un reloj y ninguna actuación arm_01. Readiness usa FQN y datos reales. | G3 |
| T05 | Comando a una instancia y comandos concurrentes | Solo se mueve el destinatario; no hay contaminación entre joint states, comandos, controles ni telemetría. | G3/G5 |
| T06 | Cuatro ocupaciones con clearance conocido | Se evalúan los seis pares; valor y primitivas limitantes coinciden con el oráculo geométrico. | G4 |
| T07 | Brazo–brazo, brazo–fixture, fixture–fixture y carga | Las herramientas/cargas están incluidas sin perder primitivas del brazo; riesgo detectado en cada pareja pertinente. | G4/G5 |
| T08 | Fuente ausente, duplicada o robot_id extra | STOP; no se acepta un conjunto de cuatro IDs arbitrarios ni dos productores de una identidad. | G4 |
| T09a | Contrato puro: identidad/estado erróneo; arrays incompletos o NaN | El validador/serializador rechaza entradas incoherentes con motivo; no exige un runtime ROS para probarlo. | G1 |
| T09b | Mensajes ROS recibidos con perfil/hash/versión/época erróneos | La frontera runtime descarta la ocupación completa antes del núcleo y publica STOP/NOT_READY; no hay mezcla parcial de revisiones. | G4 |
| T10 | Apertura/cierre de ensayo, transición, estado desconocido y carga no identificada | Envolventes correctas por estado y durante la transición, o STOP; una orden no sustituye feedback y nunca se asigna volumen cero por falta de datos. | G4 |
| T11 | TF ausente, incorrecto, tardío o sin interpolación | STOP/NOT_READY; ningún fallback silencioso a último TF ni extrapolación. | G4 |
| T12 | Snapshot fresco individualmente pero con skew excesivo | Conjunto rechazado aunque las cuatro TTL individuales no hayan expirado. | G4 |
| T13 | Ausencia, pausa, salto y reinicio de /clock | Se invalidan permisos, conjuntos y épocas; watchdog monotónico funciona; recuperación requiere datos nuevos. | G4/G5 |
| T14 | Pérdida/reinicio del coordinador o retraso del consumidor | STOP por timeout independiente; ALLOW viejo no se conserva. | G5 |
| T15 | STOP durante trayectoria aceptada, JOG, cola y replan | Se bloquea/cancela ejecución y se verifica feedback; ningún resultado tardío restablece movimiento. | G5 |
| T16 | ALLOW global con STOP/reducción local y viceversa | Gana la restricción más severa; una autoridad final emite control. | G5 |
| T17 | Caída/rechazo de acción o controlador | Readiness cae y se aplica la política del conjunto sin silenciar el fallo. | G3/G5 |
| T18 | Regresión monobrazo | Defaults, admisión, frescura, TF, watchdog, políticas y adaptador existentes conservan su comportamiento esperado. | G0/G6 |
| T19 | Escalado de primitivas/frecuencia con semillas fijas | Se registran percentiles, máximos, deadlines, edad/skew y coste de instrumentación; incumplimientos no se ocultan con promedios. | G6 |
| T20 | Clearance actual seguro con cruce futuro inseguro | Se detecta por el predictor validado, incluyendo entre muestras; no se adjudica al núcleo instantáneo. | G7 |

Los ensayos geométricos también deben cubrir igualdad exacta con el umbral STOP, separación apenas superior/inferior y segmentos degenerados; el caso límite no se acepta como clearance estrictamente seguro. En las campañas de control, el presupuesto de detección/propagación se declara antes de ejecutar: timeout de fuente + ciclo del consumidor + entrega/control asignados. Si ese presupuesto no está definido o se excede, G5/G6 no se cierran.

Métricas a registrar por sesión/robot/decisión:

- Frecuencia solicitada y efectiva; período/jitter; secuencias perdidas o repetidas.
- Edades de adquisición y recepción, skew máximo, retraso de TF y ventana de evaluación común.
- Duración del cálculo p50/p95/p99 y máximo, pares/primitivas evaluados, CPU/RSS y factor de tiempo real de Gazebo.
- Detección → decisión global → control final recibido → cancelación/hold aceptados → parada observada.
- Tasas de STOP y motivos, separando detecciones esperadas, falsas alarmas e intervalos sin datos válidos. Las falsas alarmas requieren un oráculo/escenario etiquetado.
- Coste de la instrumentación, calentamiento, duración, repeticiones y configuración del equipo/middleware.

El coste geométrico instantáneo crece como `sum(i<j) n_i*n_j` para cuatro robots; con igual número `n` de primitivas son aproximadamente `6*n²` comparaciones. Un horizonte de `H` muestras añade ese factor; TF, publicación y control también consumen tiempo y deben medirse.

`PipelineTiming` ya permite correlacionar etapas. Diferencias entre `monotonic_ns` de procesos son comparables solo si comparten reloj monotónico del mismo host/arranque; entre hosts/boots se usarán duraciones locales y un método temporal validado, no una resta de enteros no relacionados. En escenarios de geometría sin comando se necesita un ID de conjunto/decisión, no inventar un command_id.

La parada observada exige varias muestras nuevas de feedback bajo un umbral de velocidad definido por la campaña; repetir la misma muestra o pausar Gazebo no demuestra frenado. Los percentiles no son límites máximos garantizados. Los presupuestos configurables deben fijarse antes de cada campaña, relacionando adquisición, skew/TF, cálculo, comunicación, actuación y retardo de parada. Cualquier cifra inicial es de ensayo hasta disponer de resultados y márgenes justificados.

## 7. Paquetes, verificación y disciplina de implementación

| Paquete/área | Cambios previstos | Momento de compilación/verificación |
| --- | --- | --- |
| `thesis_interfaces` | Contratos runtime nuevos o ampliados tras P1. | Generar interfaces e importar defaults fail-closed; reconstruir consumidores afectados por cambios de interfaces. |
| `thesis_description` | Parametrización/selector compatible y contrato mount/TCP. | Expansión Xacro, unicidad y FK; equivalencia de variante legada. |
| `thesis_simulation` | Configuración, nombres, launch tres robots, synthetic_state, readiness y adaptador por instancia. | Tests puros y headless de una instancia antes de tres; acciones/feedback y fallos. |
| `thesis_core` | Adaptación geométrica ROS, coordinador y composición de restricciones. | Unitarias de núcleo/mensajes y runtime; control de conjunto/frescura; regresión supervisor local. |
| `thesis_telemetry` | Correlación multiinstancia y métricas de conjunto/decisión. | Tests de correlación/percentiles y medición de sobrecoste. |
| `tools/validation` | Harness específico offline con timeouts y evidencias. | Preflight de objetivos, aislamiento de dominio de ensayo y terminación solo de procesos propios. |
| `thesis_hardware` / `thesis_hardware_bridge` | Solo análisis/preparación de interfaz futura durante P0–P6. | No se inician en el launch offline; cambios de runtime físico requieren fase posterior. |

Tras retirar build/install, la reconstrucción inicial incluye el cierre de dependencias necesario, no solo un paquete Python aislado. Después se compilan los paquetes modificados y sus consumidores cuando cambie una interfaz; no se reconstruye todo el workspace por rutina. Declarar dependencias/test_depend de las pruebas nuevas. Si falta una herramienta, documentar el bloqueo; no instalar paquetes ni usar sudo sin autorización.

La aceptación integrada no puede reemplazarse por mocks que oculten el código de ROS generado o por pruebas de texto del launch. Si `launch_testing` u otra dependencia no está disponible, ese gate queda pendiente hasta disponer de la herramienta o de un harness runtime equivalente con evidencia suficiente.

Se reutilizarán selectivamente las pruebas existentes de frescura, TF, admisión, watchdog y fallos de controlador. `tools/validation/run_hardware_bridge_mock.sh` no debe ejecutarse tal cual en esta campaña: sus observaciones CLI y destinos de informes no cumplen el flujo actual, y no es el escenario offline de tres simulados.

Toda observación mediante `ros2 node/topic` usará `--no-daemon --spin-time 2`, en clientes separados y secuenciales, conforme a AGENTS.md. `tf2_echo` se ejecutará como cliente independiente y se detendrá solo ese proceso. Headless se probará desde la terminal integrada; una incidencia GUI del entorno Snap se diagnosticará antes de proponer una terminal nativa. Ninguna automatización enviará señales a launches ajenos ni buscará terminar procesos por nombre genérico.

Para cada incremento: explicar la causa, aplicar un cambio acotado, ejecutar sus pruebas y regresiones pertinentes, y guardar una evidencia breve. Usar `docs/codex_reports/` para decisiones técnicas y `evidence/codex_reports/` para pruebas/datos, con la fecha real de ejecución. No generar un informe por cada comando; una carpeta/manifiesto por campaña y un resumen por gate son suficientes.

## 8. Qué podrá completarse antes del CAD y qué dependerá de él

| Preparación posible ahora | Información o validación pendiente |
| --- | --- |
| Tres JACO2 aislados y controlables en Gazebo. | Geometría visual/collision del gripper definitivo y su posición real de montaje. |
| Mount/base/TCP reservados y transformaciones parametrizadas por instancia. | Offset TCP medido y cinemática cartesiana de la herramienta final. |
| Perfiles compuestos y estados ejercitados con fixtures sintéticos. | Número de joints, mecanismo, relaciones mimic/transmisión, actuación y apertura reales. |
| Decisiones multiarm y STOP integrados solo en simulación. | Límites físicos de fuerza/velocidad/esfuerzo y comportamiento de contacto/agarre. |
| Watchdogs, pérdida de estado/TF, reinicio y escenarios reproducibles. | Masa, centro de masa e inercia verificables para dinámica. |
| Medidas de comportamiento/rendimiento del software con un modelo de ensayo declarado. | Envolventes finales por estado/carga y comprobación de que cubren la pieza real. |

No es necesario inventar un gripper completo para terminar P0–P6. El contrato admite fixtures de envolvente de prueba y prepara los puntos de sustitución. Las dimensiones de una fixture pertenecen a su caso de prueba; no son aproximaciones validadas del gripper que falta.

## 9. Fases posteriores y dependencias externas

### F1 — Recepción e incorporación del gripper definitivo en los tres simulados

Entrada: CAD o geometría verificable, unidades/escala, montaje, articulaciones/actuación, límites y datos de dinámica si se pretende probar contactos/dinámica.

1. Importar y simplificar colisiones; verificar referencias y no duplicar mano heredada.
2. Añadir macro/variante nueva debajo de la interfaz estable; conservar namespaces y contrato por instancia.
3. Implementar controlador y feedback acordes con el mecanismo real. Reemplazar el publicador de estado sintético por estado derivado de articulaciones/feedback simulados.
4. Generar manifiesto/versiones de perfiles compuestos por estado/carga y validar coberturas. Una malla visual correcta no basta.
5. Incorporar primero en `arm_02`, validar regresión y luego replicar la misma variante en `arm_03/04` mediante configuración.
6. Repetir T03, T07, T10 y las pruebas de control/parada/rendimiento afectadas; validar cartesiano/TCP si se habilita.

Se podrá comenzar por geometría rígida y análisis de ocupación si solo hay formas/dimensiones verificadas. Las conclusiones de agarre, esfuerzos y dinámica se posponen hasta contar con sus datos.

### F2 — JACO2 físico observacional, futuro y opcional para el cierre pregripper

Requiere solicitud/autorización aplicable a conexión física según AGENTS.md. La primera observación seguirá el launch existente de estado con `hardware_output=false` y `hardware_output_enabled=false` efectivo, comprobando conexión, diagnóstico y estado sin movimiento.

El launch de observación no inicia productores de comandos, pero el mismo `jaco_hardware_node` mantiene suscripciones/servicios de control en su implementación. Por tanto, no se supondrá que el nombre read_only elimina esos endpoints: se mantendrá la salida deshabilitada y se comprobarán aislamiento y ausencia de conexión de órdenes desde el escenario simulado. La primera campaña puede registrar por separado y analizar offline, sin exigir un puente de reloj vivo.

Antes de sustituir `synthetic_state` en una evaluación híbrida se necesitan el volumen medido del conjunto físico, un `world` físico/simulado calibrado y una adaptación temporal explícita con incertidumbre, épocas y comportamiento ante pausa/saltos. No bastan `real_time_factor≈1` ni volver a fechar mensajes al recibirlos. Se preservan los sellos de adquisición originales. La fuente sintética y la física no coexistirán usando la misma identidad.

F2 puede desarrollarse independientemente de recibir el CAD de los grippers simulados; lo que sí exige es conocer la geometría del JACO2 físico realmente presente y su herramienta averiada. La observación por sí sola no habilita actuación ni prueba una parada física global.

### F3 — Coordinación híbrida con movimiento físico

Es una etapa posterior distinta. Requiere validar antes toda la cadena de decisión/timeout/actuación hacia el bridge, mantener las restricciones locales y demostrar el presupuesto de reacción con la geometría física. Cualquier prueba que pueda mover el brazo requiere las confirmaciones y pasos de AGENTS.md: operador autorizado, área despejada, parada de emergencia accesible, verificación inicial sin movimiento y confirmación antes de cada etapa de desplazamiento mínimo a velocidad reducida.

Los cuerpos virtuales de tres brazos permiten provocar casos de software frente al JACO2 real, pero no reproducen interacciones mecánicas entre cuatro robots físicos. Los resultados se presentarán como experimento híbrido de la capa de seguridad con sus fuentes y modelos declarados.

## 10. Dependencias, paralelismo y punto de cierre

```text
P0 baseline → P1 contratos
                 ├─ P2 una instancia parametrizada → P3 tres simulados + arm_01 sintético
                 └─ P4 núcleo/adaptadores puros (en paralelo; integración tras G3)
                                          ↓
                               G4 coordinador observacional
                                          ↓
                               P5 STOP solo simulación
                                          ↓
                               P6 campaña / cierre sin CAD

P7 predicción: contrato/núcleo tras G4; validación de control tras G5
F1 CAD: incorporación primero a arm_02 y después a arm_03/04
F2 observación física: rama futura independiente, sin requisito para G6
F3 movimiento físico: gate posterior con nuevas verificaciones/autorización
```

P1 se cerrará antes de repartir cambios en mensajes, nombres y launch. P2/P3 y el trabajo puro de P4 pueden avanzar en paralelo con propietarios de archivos distintos; la integración ROS P4 exige G3. Las pruebas y la instrumentación acompañarán cada cambio. P5 no empezará a controlar movimiento hasta validar G4 en observación.

El primer lote implementable será P0–P1 y una instancia `arm_02` de P2, como cambio revisable con sus pruebas. El siguiente lote replicará esa instancia a tres y añadirá la cuarta fuente lógica. No conviene implementar simultáneamente launch, gripper ficticio, nueva cinemática cartesiana y bridge físico: no habría un gate intermedio que localice fallos.

El cierre verificable del trabajo previo será G6: tres brazos simulados operativos, una cuarta fuente sintética trazable, integración instantánea multiarm y STOP exclusivamente de simulación, interfaces de herramienta ensayadas y un manifiesto concreto de lo que aportará el CAD. G7, F1 y F2/F3 conservarán estados independientes para no presentar como completadas validaciones que aún dependen de otro modelo o del hardware.

## 11. Resultado de este turno

Se realizó planeamiento con inspección de código y discusión entre tres agentes, seguida de una ronda de reconciliación de topología/alcance y revisión cruzada del documento. Se incorporaron sus precisiones sobre el binding del plugin ros2_control por instancia, autoridad única de reloj, validación previa al núcleo y criterios verificables de G0/G1; el agente principal añadió la corrección de base FK, contrastada con el agente de seguridad. Solo se creó este documento. No se modificó código/configuración, no se compiló ni se repitieron pruebas, y no se inició ROS, Gazebo o el JACO2 físico. Los 37 resultados de pruebas citados corresponden a la auditoría previa de la misma sesión.
