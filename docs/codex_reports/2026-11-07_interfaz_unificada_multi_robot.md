# Interfaz unificada de simulación para cuatro brazos

Fecha: 2026-11-07

## Resultado

Se incorporó una interfaz Qt guiada por el manifiesto de ejecución de
`thesis_multi_robot`. La misma ventana funciona con JACO2, UR17g-1300,
TM25S y P3020, tanto en una célula de cuatro perfiles como en lanzamientos
aislados. La cantidad de articulaciones, nombres, límites, estado de
proximidad y cinemática se obtienen del perfil activo; no hay valores de un
fabricante codificados en la vista.

La ruta histórica `thesis_simulation` + `thesis_ui` del JACO2 no fue
reemplazada ni modificada. La nueva interfaz pertenece a
`thesis_multi_robot` y es exclusivamente de simulación.

## Arquitectura implementada

- `multi_robot_gui.py`: ventana Qt, selector de perfil, tabla dinámica de
  articulaciones, incrementos, IK puntual y feedback.
- `multi_robot_gui_node.py`: suscripciones namespaced, manifiesto/URDF,
  límites, frescura, IK y publicación de candidatos.
- `cell.launch.py`: argumento `start_gui`; una sola GUI por manifiesto.
- `profile_supervisor`: publica `SIMULATION_GOAL_SUCCEEDED` al recibir el
  resultado terminal exitoso de `FollowJointTrajectory`.
- `validate_multi_robot_gui.py`: validación reproducible de célula completa o
  de un perfil aislado.

La GUI nunca publica en un controlador. Solo usa
`/<perfil>/thesis/candidate_command`; la decisión y el movimiento permanecen
en el supervisor y `/<perfil>/arm_controller`.

## Correcciones derivadas de la auditoría multiagente

1. Se eliminó una colisión con el atributo interno `_publishers` de `rclpy`.
2. `use_sim_time` se aplica al construir el nodo, antes de validar timestamps.
3. Los objetivos de la ventana se invalidan al cambiar de perfil y se copian
   únicamente desde el primer estado fresco. No es posible enviar ceros no
   inicializados durante el arranque.
4. `REACHED` exige dos condiciones: resultado terminal
   `SIMULATION_GOAL_SUCCEEDED` y error articular máximo menor o igual a
   0,008 rad. Esto evita enviar un retorno mientras el supervisor sigue
   ocupado con la acción anterior.
5. La carga de modelos se serializó como
   `spawn → joint_state_broadcaster → arm_controller` por perfil. La carga
   simultánea podía bloquear Gazebo durante el descubrimiento del servicio de
   `robot_state_publisher`.
6. El validador comprueba por servicio ambos controladores activos antes de
   ejercer la interfaz.

## Alcance y límite de paridad

La paridad nominal común incluye estado articular, objetivos articulares,
incrementos pequeños, IK puntual, feedback de proximidad/decisión/seguimiento
y comportamiento fail-closed ante estado caduco o `STOP`.

No se trasladaron a los otros perfiles las funciones avanzadas de la GUI
histórica del JACO2: JOG continuo con watchdog, trayectorias cartesianas
rectas largas y cancelación manual, readiness integral histórica, percepción
RGB-D y MoveIt. Esas funciones exigirían nuevos contratos y validación por
robot; simularlas mediante publicaciones repetidas no sería equivalente ni
seguro. La GUI histórica del JACO2 continúa disponible para ese alcance.

## Operación

Los comandos exactos para abrir UR17g, TM25S, P3020, JACO2 o la célula
completa se conservan en `src/thesis_multi_robot/README.md`. Cada ejecución
debe usar una sola instancia por dominio/partición. No se habilitó ni se probó
hardware físico.
