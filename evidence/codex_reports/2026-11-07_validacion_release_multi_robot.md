# Validación de simulación multi-robot

Fecha: 2026-11-07

## Alcance

Se validó exclusivamente la ruta de simulación Gazebo Fortress para JACO2,
UR17g-1300, TM25S y P3020. No se iniciaron drivers de hardware, no se habilitó
`hardware_output` y no se envió ningún comando al JACO2 físico.

Tres auditorías independientes revisaron los launches y dependencias, la
supervisión y los modelos digitales. No encontraron bloqueos para la campaña
nominal de simulación.

## Resultados

- Compilación selectiva: 6 paquetes terminados correctamente.
- Pruebas de `thesis_multi_robot`: 74 aprobadas, 0 fallos.
- Regresión funcional `thesis_core`: 173 aprobadas.
- Regresión funcional `thesis_simulation`: 41 aprobadas.
- Célula completa: `pass: true`.
- Perfiles aislados JACO2, UR17g, TM25S y P3020: `pass: true` en los cuatro.
- La herramienta `command_robot.py` fue aceptada por el supervisor en los
  cuatro perfiles usando la descripción activa del simulador.
- La GUI de Gazebo arrancó, creó los cuatro robots y cargó sus controladores.
  La prueba se detuvo de forma intencional a los 20 segundos.
- Tras las pruebas no quedaron procesos de simulación activos.

La validación de célula completa comprobó estados frescos, estado `ALLOW`,
controladores activos, coherencia FK/TF, movimiento supervisado de `+0.02 rad`
y retorno, cancelación por obstáculo con deriva de hold de aproximadamente
`0.000001 rad`, rechazo por solapamiento, rechazo de fuente caducada y cuatro
movimientos concurrentes de `+0.01 rad`.

## Evidencia reproducible

- `2026-11-07_multi_robot_release_validation/all_result.json`
- `2026-11-07_multi_robot_release_jaco2/jaco2_result.json`
- `2026-11-07_multi_robot_release_ur17g/ur17g_result.json`
- `2026-11-07_multi_robot_release_tm25s/tm25s_result.json`
- `2026-11-07_multi_robot_release_p3020/p3020_result.json`

Cada carpeta incluye también el log completo del launch correspondiente.

## Observaciones no bloqueantes

- Gazebo muestra advertencias de materiales ausentes en partes del OBJ oficial
  del TM25S. Las geometrías se cargan y el problema no afectó el control ni las
  colisiones.
- El linter global histórico recorre todo el repositorio y los submódulos de
  fabricante. Al limitarlo a la integración nueva se detectaron 93 avisos de
  Flake8 y 15 de PEP257, principalmente longitud de línea y formato de
  docstrings. Se registra como deuda de estilo; las pruebas funcionales y de
  simulación permanecen aprobadas.
- La supervisión sigue siendo experimental. No cubre autocolisión, retiming de
  `REDUCTION`, frenado dinámico certificado, objetivos futuros compartidos entre
  brazos ni trayectorias cartesianas rectas.
- `accepted=true` en la herramienta de operador indica admisión por el
  supervisor, no confirmación de fin de trayectoria. El validador integrado sí
  observa la posición alcanzada y la deriva posterior a una cancelación.

## Veredicto

Los cuatro perfiles están listos para pruebas nominales, controladas y de baja
amplitud en simulación. Esto no constituye certificación de seguridad ni
equivalencia completa con el pipeline histórico del JACO2.
