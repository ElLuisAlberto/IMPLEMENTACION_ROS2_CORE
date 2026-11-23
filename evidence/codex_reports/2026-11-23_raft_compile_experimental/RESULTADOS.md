# Fase experimental RAFT compilado

Fecha: 23 de noviembre de 2026.

## Resultado

Se agregó una variante separada de RAFT que conserva los pesos, la resolución
256×192, el tipo Float32 y las 12 iteraciones actuales. El nodo nuevo publica
exclusivamente en `/thesis/perception/point_flow_raft_preview_experimental`.
El nodo legacy y sus tópicos no fueron reemplazados.

## Medición aislada

Benchmark de inferencia sin ROS ni RViz:

| Variante | Mediana | FPS teórico |
|---|---:|---:|
| eager | 21.87 ms | 45.7 |
| torch.compile reduce-overhead | 16.66 ms | 60.0 |

La ejecución live de ambas variantes mostró:

| Variante | Frecuencia recibida |
|---|---:|
| legacy | 15.88 Hz |
| experimental | 18.64 Hz |

La mejora observada en ejecución conjunta fue 17.4%. La variante experimental
mantuvo el flujo final en aproximadamente 20 ms después del calentamiento
frente a aproximadamente 28 ms del legacy.

## Comparación numérica

Usando 20 pares consecutivos reales del bag
`tesis_point_flow_live_20261123`:

- diferencia máxima: `0.00007413`
- diferencia media máxima: `0.00000382`

La prueba quedó dentro de una tolerancia Float32 de `0.001`. Esta diferencia
no se tradujo en cambios de resolución, tipo de datos, pesos ni iteraciones.

## Validaciones

- Python `py_compile`: aprobado.
- `colcon build --packages-select thesis_perception`: aprobado.
- Smoke test eager vs compilado sobre frames reales: aprobado.
- Ejecución simultánea con el nodo legacy: aprobada.
- Ejecución con cámara real: aprobada en observación.
- No se inició movimiento ni control físico del JACO2.

## Conclusión

La variante compilada mostró una mejora medible y estable sin alterar resolución,
pesos, precisión FP32 ni número de iteraciones. Debe mantenerse separada hasta
completar campañas adicionales de movimiento lento y rápido. La prioridad
siguiente es un smoke test C++/LibTorch o TensorRT si se desea una integración
permanente, pero no es necesaria para conservar el resultado actual.
