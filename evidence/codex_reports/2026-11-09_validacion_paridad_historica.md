# Validación de paridad histórica multi-brazo

Fecha: 2026-11-09

## Compilación y regresión

Paquetes compilados con `--symlink-install`:

```text
thesis_core
thesis_ui
thesis_simulation
thesis_multi_robot
```

Resultado: 4/4 paquetes compilados.

La regresión conjunta se ejecutó con:

```bash
colcon test --packages-select \
  thesis_core thesis_ui thesis_simulation thesis_multi_robot \
  --event-handlers console_cohesion+ --return-code-on-test-failure
colcon test-result --test-result-base build --verbose
```

Resultado consolidado:

```text
372 tests, 0 errors, 0 failures, 3 skipped
```

Desglose de pytest:

| Paquete | Aprobadas | Omitidas | Fallos |
| --- | ---: | ---: | ---: |
| thesis_core | 194 | 1 | 0 |
| thesis_ui | 31 | 1 | 0 |
| thesis_simulation | 53 | 1 | 0 |
| thesis_multi_robot | 91 | 0 | 0 |

Las omisiones corresponden a comprobaciones de copyright; no son pruebas
funcionales fallidas.

## Validación viva independiente

Cada brazo se inició solo, en un dominio ROS y partición Gazebo aislados. Se
usó un obstáculo demo lejano y un incremento pequeño de J1 de `0.01 rad`. El
validador esperó readiness, coherencia FK-TF, proximidad, proyección futura,
decisión del supervisor, aceptación del controlador, finalización y estado
articular final.

| Brazo | DOF | Readiness | FK-TF | Decisión | Ejecución | Predicción |
| --- | ---: | --- | --- | --- | --- | --- |
| JACO2 | 6 | READY | VALID, error 0.000000 m | ALLOW/CLEAR | PENDING, ACCEPTED, SUCCEEDED | 21 muestras, execution |
| UR17g | 6 | READY | VALID, error 0.000000 m | ALLOW/CLEAR | PENDING, ACCEPTED, SUCCEEDED | 21 muestras, execution |
| TM25S | 6 | READY | VALID, error 0.000000 m | ALLOW/CLEAR | PENDING, ACCEPTED, SUCCEEDED | 21 muestras, execution |
| P3020 | 5 | READY | VALID, error 0.000000 m | ALLOW/CLEAR | PENDING, ACCEPTED, SUCCEEDED | 21 muestras, execution |

Clearance reportado por el monitor con el obstáculo lejano:

| Brazo | Clearance mínimo (m) |
| --- | ---: |
| JACO2 | 7.7877974993 |
| UR17g | 7.7776280218 |
| TM25S | 7.4332506696 |
| P3020 | 7.6235254019 |

Los cuatro perfiles publicaron predicción de seguridad `ALLOW` y los
namespaces de marcadores esperados:

```text
current_capsules
horizon_endpoint_traces
horizon_labels
horizon_time_poses
predicted_volume
```

## GUI, seguridad y geometría

- La GUI Qt real abrió en modo offscreen para los cuatro perfiles sin
  traceback.
- JACO2 mantuvo 6 filas y la postura histórica `0/180/180/0/0/0` grados.
- UR17g y TM25S mostraron 6 filas; P3020 mostró 5.
- Un perfil no JACO2 en modo hardware fue rechazado de forma explícita.
- UR17g usa 8 cápsulas, TM25S 7 y P3020 22. Monitor, supervisor y preview
  produjeron la misma geometría y el mismo clearance para cada perfil.
- En P3020, una traslación de 5 mm resolvió con cinco articulaciones; una
  rotación explícita no alcanzable fue rechazada. En modo solo posición la
  orientación puede variar, como corresponde a un manipulador de 5 DOF.
- Los indicadores de conexión fueron comprobados con nombres remapeados y el
  servicio de armado también está incluido en el contrato namespaced, aunque
  esta ruta permanece exclusivamente en simulación.

## Preservación del JACO2

La auditoría detectó que cargar JACO2 mediante el manifiesto genérico habría
usado 13 cápsulas en lugar de las 6 históricas. Se corrigió el wrapper para
bifurcar `robot:=jaco2` directamente al launch original. La validación viva de
JACO2 posterior a ese cambio terminó en `SUCCEEDED` y conservó sus nombres de
articulación globales.

## Observaciones

Durante el arranque de P3020 hubo un timeout transitorio de consulta al
controller manager; el spawner se recuperó, el sistema alcanzó `READY` y toda
la secuencia terminó correctamente. En algunos cierres aparecieron avisos de
corutinas/Future de `rclpy` posteriores a `Ctrl+C`; no afectaron ejecución ni
dejaron procesos ROS, Gazebo o RViz activos.

Las comprobaciones automáticas validaron publicaciones y marcadores para RViz;
no se guardó una captura visual de la ventana desde el entorno integrado.
