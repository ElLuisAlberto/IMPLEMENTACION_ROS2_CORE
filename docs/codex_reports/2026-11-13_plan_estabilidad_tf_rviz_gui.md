# Plan de estabilidad temporal, RViz y procesamiento de la GUI

Fecha: 2026-11-13, Asia/Manila.
Estado: **planeamiento completado; implementación pendiente**.
Revisión: agente principal y tres agentes de temporalidad/control, representación
visual y regresiones/GUI. En este turno no se ejecutaron clientes ROS, pruebas,
movimientos ni cambios de parámetros o código. La sesión abierta quedó intacta.

## Objetivo y evidencia de partida

Eliminar los STOP intermitentes evitables por llegada tardía de TF y las
representaciones que confunden falta de datos con proximidad real. Mantener
las funciones de control y sus restricciones, y verificar capacidad de respuesta
con Gazebo, RViz y GUI abiertos simultáneamente.

El diagnóstico previo midió RSP a 20 Hz, obstáculo a unos 30 Hz, plazo TF total
de 40 ms y esperas observadas de hasta 61.158 ms. En una ventana distinta hubo
47 TF_UNAVAILABLE de 193 estados, con FK-TF válido y error cero. No se atribuye
cada espera a un fallo concreto: son ventanas diagnósticas separadas.

Referencia: `2026-11-13_diagnostico_nodos_sesion_jaco_abierta.md`, en esta carpeta.
La última regresión documentada tiene 413 pruebas funcionales aprobadas; este
planeamiento no las repite ni afirma que los cambios propuestos estén validados.

## Contratos que deben mantenerse

- Consulta de TF al sello exacto de adquisición, sin sustituir por último TF,
  retrasar artificialmente el sello ni volver a fechar datos antiguos.
- Presupuesto total de 40 ms compartido entre las consultas del monitor,
  geometría, radios y umbrales actuales. Falta de datos conserva STOP y
  `geometry_valid=false`; ninguna visualización concede autorización.
- Ruta de comandos, arbitraje, watchdogs, cancelación, armado y cierre actuales.
- Predicción funcional del horizonte: H=1 s, N=21, radios, muestreo y publicación
  de `/thesis/joint_trajectory_prediction`. Ocultar una capa de RViz no significa
  apagar su nodo ni modificar los datos que consume el supervisor.
- Fuente, reloj y nombres de cada brazo. El híbrido tiene tres actuadores de
  Gazebo y una identidad sintética, no cuatro actuadores.
- Cambios existentes del workspace, archivos y evidencias anteriores.

## Secuencia y criterios de salida

| Etapa | Trabajo | Criterio antes de avanzar |
| --- | --- | --- |
| P0 | Registrar baseline, manifiesto y condiciones de carga | Reproducción comparable y cambios previos identificados |
| P1 | Aumentar producción TF del monobrazo | Llegada TF compatible con 40 ms y fallos cerrados conservados |
| P2 | Validez, caducidad y limpieza de marcadores RViz | Ningún dato desconocido o histórico presentado como actual |
| P3 | Lotes de callbacks GUI con presupuesto | Interfaz fluida sin alterar órdenes, JOG ni cierre |
| P4 | Regresión e integración de todos los cambios | Pruebas anteriores y nuevas aprobadas, campaña gráfica completa |
| P5 | Entrega y procedimiento de retorno | Comandos, perfiles, límites y evidencias reproducibles |

Se verifica cada parche por separado para atribuir sus efectos. Si P1 no cumple
por carga, se detienen sus pruebas de movimiento y se investiga; P2/P3 pueden
avanzar aisladamente como mejoras de presentación y consumo, pero no se declara
cerrada la integración hasta repetir el gate temporal completo.

### P0 — Baseline y reversibilidad

1. Registrar HEAD, diff, hashes y copias de los archivos afectados, incluyendo
   modificaciones no rastreadas relevantes. Un commit por sí solo no representa
   el estado de este workspace.
2. Capturar la misma postura, obstáculo, configuración de control, perfiles y
   programas gráficos. Medir tras 20 s de calentamiento en tres ventanas de 60 s.
3. Obtener frecuencia efectiva TF en tiempo ROS y de pared, frecuencia de
   articulaciones/obstáculo, edad de sellos, esperas TF, proporción de
   `geometry_valid`, razones de STOP, readiness, RTF y carga CPU.
4. Usar observadores de lectura consecutivos. Toda consulta CLI de nodos/tópicos
   mantiene `--no-daemon --spin-time 2`; una lista parcial al arrancar no basta
   para diagnosticar nodos ausentes.
5. Para comparar, evitar dos simulaciones completas simultáneas que alteren la
   carga. Coordinar la transición a una sesión de prueba propia antes de la
   implementación; no detener indiscriminadamente la sesión del operador.

### P1 — Producción TF suficiente

Archivo principal: `src/thesis_simulation/launch/jaco_gazebo.launch.py`.

1. Configurar explícitamente `robot_state_publisher.publish_frequency=100.0`,
   coherente con el feedback configurado a 100 Hz. Facilitar comparación con
   20/50 Hz mediante configuración de lanzamiento si resulta útil.
2. Confirmar frecuencia efectiva: es un límite máximo, no crea muestras si los
   estados articulares llegan a menor frecuencia.
3. Comparar baseline y candidato con las tres aplicaciones gráficas abiertas.
   Objetivos propuestos, aún por verificar:
   - Cero TF_UNAVAILABLE evitables en las tres ventanas sanas estabilizadas.
   - p99 de espera TF ≤30 ms como objetivo de margen frente al límite de 40 ms.
   - FK-TF válido, distancias coherentes y sin regresión sostenida de respuesta.
4. La llegada a un observador es una aproximación al tiempo de disponibilidad:
   contrastarla siempre con los resultados reales del buffer/monitor y sus
   sellos. Un STOP geométrico válido no cuenta como fallo temporal evitable.
5. Inyectar pérdida de TF y de fuente en la sesión propia: conservar STOP,
   geometría inválida y bloqueo de órdenes. Recuperar solo con evidencia válida.
6. Si 100 Hz empeora la carga, evaluar 50 Hz con los mismos criterios. No
   aceptarlo únicamente por consumir menos CPU. Si ambos fallan, examinar DDS,
   reloj de 1 kHz y trabajo gráfico; no ampliar tolerancias ni cambiar física
   como primera medida.

Pruebas focales: TF exacto, listener independiente, presupuesto global de las
consultas, ausencia de fallback, fuente inválida, identidad y readiness.
Compilar inicialmente solo `thesis_simulation` si no se modifica otro paquete.

No extender automáticamente el parámetro al híbrido: sus RSP son independientes
y la fuente sintética publica a 50 Hz. Primero medir esas rutas.

### P2 — RViz coherente con la vigencia de los datos

Archivos: `capsule_visualizer.py`, funciones exclusivamente visuales de
`horizon_preview.py`, perfil nuevo de RViz y pruebas focales.

1. Guardar sello original y recepción monotónica. Exigir geometría válida,
   referencia compatible, segmento reconocido, valores finitos y radio positivo
   antes de mostrar una distancia actual. Propuesta inicial de vigencia de
   proximidad: 0.5 s; comprobarla contra frecuencias y pausas antes de fijarla.
2. Ante TF/fuente ausentes o caducidad, retirar la distancia numérica, testigo y
   resaltado anterior. Mostrar «STOP preventivo — geometría no disponible», con
   STOP explícito y geometría neutra. Desconocido nunca se convierte en ALLOW;
   actualmente el horizonte puede volver a ALLOW al vencer la proximidad.
3. Mantener conjunto anterior y actual de `(namespace,id)` y emitir DELETE para
   sus diferencias, también si no queda ninguna cápsula por falta de TF.
   Añadir vida finita como respaldo, inicialmente 0.5 s, y limpieza monotónica
   que contemple reloj pausado o reiniciado. Borrar solo IDs propios; no afectar
   otros productores con un DELETEALL indiscriminado.
4. Separar candidato de ejecución actual. Mostrar ID y «CANDIDATO» y retirarlo
   tras un plazo explícito corto, inicialmente 1 s, o al invalidarse el contexto.
   Una historia opcional tendrá namespace, etiqueta y toggle propio, apagado
   inicialmente. No borrar registros guardados ni colorear el brazo actual con
   una predicción histórica.
5. Para la distancia actual, dibujar entre superficies usando los radios del
   mismo modelo/medición. Conservar separación firmada y tratar contacto,
   solapamiento y centros coincidentes sin divisiones por cero.
   `TrajectoryPrediction` no tiene radio del obstáculo: en el testigo futuro
   conservar eje-centro rotulado hasta disponer de procedencia suficiente. No
   tomar el radio de otra medición ni ampliar mensajes ROS solo para este ajuste.
6. Crear perfil de operación sencillo, conservando el perfil anterior como
   referencia. Mostrar robot, un juego de cápsulas, un obstáculo y su distancia
   vigente. Dejar trazas, etiquetas extensas, posturas futuras y volumen avanzado
   disponibles pero desactivados inicialmente; evitar esfera duplicada.

Validación: actual→inválido→recuperado; mensajes vencidos/atrasados; cero real;
TF parcial/total; eliminación con salida vacía; pausa/reinicio; candidato viejo;
testigo geométrico y valores no finitos. Comparar los mensajes de predicción y
decisión antes/después: el cambio visual no debe alterar sus contenidos.

Compilar `thesis_core`, `thesis_description` y `thesis_simulation` únicamente si
esas fases modifican sus fuentes, perfiles o selección de perfil.

### P3 — GUI con capacidad de atender sus suscripciones

Archivo principal: `src/thesis_ui/thesis_ui/joint_control_gui.py`.

1. Conservar el hilo Qt y los temporizadores actuales: ROS 10 ms, refresco
   100 ms y JOG 50 ms. No introducir actualización concurrente de widgets.
2. Procesar un lote limitado por cantidad y tiempo monotónico, con llamadas
   no bloqueantes. Punto inicial de evaluación: 16 callbacks o 2 ms por ciclo;
   sus valores definitivos dependen de carga medida, no son una garantía.
3. Comprobar contexto/cierre entre callbacks, cola vacía y excepciones reales.
4. Primer parche sin cambios de QoS. Conservar colas y semántica de decisiones,
   ejecución, controles, cancelación y armado. Si hace falta reducir colas de
   telemetría, hacerlo como cambio separado y probar ausencia de pérdidas
   funcionales; no aplicar indiscriminadamente profundidad uno a eventos.
5. Medir edad desde sellos de origen además del recibo al procesar callback:
   una cola atrasada no debe parecer nueva porque acaba de ejecutarse su callback.

Objetivos de integración propuestos: refresco/respuesta visual p95 ≤150 ms, edad
de telemetría sin crecimiento sostenido y decisiones sin inanición. Medir jitter
JOG p95/p99 frente a sus watchdogs vigentes; no ampliar esos watchdogs para pasar.

Regresión: límites de lote/tiempo, apagado durante lote, cola vacía, ejecución
PENDING/ACCEPTED/SUCCEEDED/CANCELED, rechazo ajeno, locks, JOG deliberado,
navegación, pérdida de foco, cierre, HOLD/cero y ausencia de reanudación al volver.
Conservar equivalencia de solicitudes entre workspace y legacy.
Compilar únicamente `thesis_ui` en esta fase.

### P4 — Integración y pruebas por escenario

1. Repetir las suites funcionales previas por paquete y añadir las regresiones
   nuevas. No sustituir los 413 casos anteriores por tests solo del parche.
2. Comparación gráfica completa, calentamiento y tres ventanas de 60 s por
   configuración aceptada. Registrar p95/p99/máximo, no solo promedios.
3. En simulación propia: reposo ALLOW; WARNING, REDUCTION y STOP geométricos;
   objetivo articular pequeño; JOG con HOLD; objetivo cartesiano aplicable;
   cancelación, pérdida TF/fuente, pausa y recuperación. No reanudar órdenes
   terminadas o canceladas automáticamente.
4. Verificar cada instancia `arm_02–04` y el tablero conjunto, con sus propios
   prefijos/bases/frames. `arm_01` conserva su papel sintético sin endpoint físico.
   La cobertura de geometría/tablero no se declara como control de cuatro brazos.
5. Probar ventanas auxiliares y cierre principal, sin publicar mandos adicionales
   ni crear una segunda autoridad de comandos por abrir un panel.

Fallo en cualquier contrato de seguridad o regresión funcional deja la fase
pendiente. No se oculta ni se reduce el criterio por haber mejorado el aspecto.

### P5 — Entrega y retorno

- Publicar evidencia de build, pruebas, mediciones y capturas en
  `evidence/codex_reports/`; mantener diagnósticos/planes en esta carpeta.
- Documentar valores elegidos, perfil operativo/avanzado y comandos de arranque.
- Conservar copias/manifiestos y parches de cada fase. Ante regresión, retirar
  solo el parche propio de esa fase o seleccionar el perfil previo, preservando
  las modificaciones existentes; no usar reset/checkout/stash destructivos.
- Detener únicamente los procesos propios de la campaña y dejar claro qué
  sesión queda abierta. No usar sudo ni instalar dependencias sin autorización.

## Alcance físico y estado de ejecución

Todo este plan se ejecutará inicialmente en simulación. Los módulos compartidos
de GUI tendrán regresiones de armado/cierre mediante dobles de prueba; eso no
constituye validación física. Cualquier prueba real sigue las autorizaciones y
confirmaciones por etapa del AGENTS.md.

El diagnóstico y este plan están terminados. Los cambios de frecuencia, RViz y
lotes de GUI descritos aquí todavía no se han implementado ni validado.
