# Diagnóstico de aparente falsa proximidad en simulación

Fecha: 2026-11-13, Asia/Manila. Campaña iniciada el día anterior.
Alcance: simulación, interfaz y cuatro identidades lógicas. Sin hardware Kinova.

## Causas verificadas y correcciones

1. El launch completo elegía `topic` y publicaba un obstáculo demo fijo. Mover
   la esfera visible de Gazebo no movía esa estimación. Se reprodujo una
   discrepancia de 1.4 m: escena en `(2,0,0.65)` y monitor en `(0.6,0,0.65)`.
   Ahora el modo interactivo predeterminado es `gazebo`; `topic` sigue siendo
   explícito para estimaciones. El demo y su puente solo arrancan en `topic`.
2. Un fallo TF publicaba STOP con distancia cero y geometría válida; la GUI
   mostraba ese cero como una medida de contacto. Ahora TF/fuente ausentes
   marcan geometría inválida y la GUI muestra «Sin geometría válida», sin
   relajar STOP. El contacto real a cero y el solapamiento siguen siendo medidas.
3. El SceneBroadcaster de Gazebo fecha el encabezado de Pose_V, pero no cada
   pose. El bridge instalado producía `stamp=(0,0)` en la transformación de la
   esfera, aun con reloj simulado activo. El fallback antiguo ocultaba el fallo.
   Se añadió el plugin ya instalado PosePublisher al obstáculo, a 30 Hz. Este
   fecha cada pose con tiempo de adquisición simulado. El tópico Gazebo
   `/model/safety_obstacle/pose` se puentea al tópico ROS existente
   `/world/jaco_world/pose/info`. No se inventan fechas de recepción.
4. Si faltaba o vencía la pose Gazebo, el monitor volvía a la posición inicial y
   le asignaba fecha nueva. Ahora publica STOP / OBSTACLE_UNAVAILABLE y mantiene
   geometría inválida hasta recuperar evidencia válida. Se comprueban edad ROS,
   sello y recencia monotónica. La velocidad usa tiempo de adquisición simulado.
5. La esfera SDF tenía radio fijo 0.12 m aunque se cambiara `obstacle_radius`.
   Ahora el SDF se construye en memoria con radio único para visual, colisión y
   monitor. El archivo original no se sobrescribe. En modo `topic` no se crea
   una esfera Gazebo ajena a la estimación; RViz representa el obstáculo evaluado.
6. El readiness consideraba suficiente recibir cualquier mensaje, incluso una
   geometría inválida. Ahora informa WAITING ante ese fallo. Un STOP geométrico
   válido sigue siendo disponibilidad de datos, no permiso de movimiento.
7. Las métricas JOG articular comparaban un ID literal que no era el ID activo
   generado por la GUI. Ahora muestran solo la proyección del flujo activo.

También se restringió la retirada protectiva de la GUI a geometría válida y
radio positivo. La ausencia de fuente no habilita recuperación. Se eliminaron
warnings repetidos por cada evaluación de un fallo persistente, manteniendo
la publicación periódica de STOP.

La solución de sellos se contrastó con el
[código oficial de PosePublisher para Fortress](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)
y se verificó con los mensajes del plugin instalado. No se instalaron paquetes.

## Brazos y límites del modelo

Los cuatro bloques previamente entregados ejecutan el mismo monobrazo en
dominios distintos: comparten las correcciones anteriores. No seleccionan cuatro
herramientas o modelos distintos.

En el escenario híbrido real, `arm_01` es sintético y no tiene actuador;
`arm_02–04` son Gazebo. Se compararon URDF y FK para cuatro bases, sus yaw, los
seis ejes de cápsulas y efector, en postura inicial y otra no trivial. Error
máximo de las comparaciones: 2.22e-16. Sin doble traslación ni yaw omitido.
La verificación runtime de seis pares coincidió con ese cálculo.

Las distancias son entre cápsulas conservadoras y esfera, no entre mallas. No
se redujeron radios del brazo ni umbrales de protección. La incertidumbre del
modo topic sigue sumándose al radio de la estimación. La ruta física conserva
su selección explícita de fuente; no se declara validación física ni del gripper.

Resultados: `evidence/codex_reports/2026-11-13_validacion_proximidad_simulada.md`.
