# Reconstrucción emparejada de barridos JOG

Se procesaron las tres trazas `2026-11-18_jaco_probe_trazas/jaco_tf_100hz_96{a,b,c}.jsonl` con `tools/validation/analyze_jog_trace.py`. Para cada decisión REDUCTION se emparejaron intención, escala y la última muestra articular recibida antes de la intención; se reconstruyeron los dos horizontes con esa misma postura, duración de 1 s, 21 muestras y geometría de cápsulas. La antigüedad de la postura seleccionada fue como máximo 30,7 ms. Esa postura no necesariamente coincide exactamente con la muestra usada internamente por el supervisor, por lo que se trata de una reconstrucción, no de una captura interna exacta.

| Sesión | Pares | Barrido nominal mediano | Barrido supervisado mediano | Velocidad mediana del eje 1 |
| --- | ---: | ---: | ---: | ---: |
| a | 448 | 6,424 mm | 0,802 mm | 0,00780 rad/s |
| b | 451 | 6,424 mm | 0,813 mm | 0,00753 rad/s |
| c | 452 | 6,424 mm | 0,790 mm | 0,00770 rad/s |

Todos los pares reconstruidos presentaron un recorrido máximo de extremos menor en el horizonte supervisado. La comparación se refiere al máximo desplazamiento de cada extremo desde su posición inicial, no al volumen geométrico de la envolvente ni a la imagen mostrada en RViz. La mediana libre anterior fue 0,06466 rad/s para el mismo eje y mando; la ventana de cálculo no es idéntica a la de esta tabla.

Posteriormente se instrumentó el supervisor para publicar, bajo demanda, la postura y el objetivo efectivamente usados por cada decisión JOG. En las sesiones `jaco_exact_horizon_96.jsonl`, `jaco_exact_horizon_96b.jsonl` y `jaco_exact_horizon_96c.jsonl`, la reconstrucción con esos valores exactos y la misma duración produjo respectivamente 451, 451 y 450 pares REDUCTION: todos se contrajeron. La mediana nominal fue 6,4235 mm en las tres y la supervisada 0,7996, 0,8096 y 0,8374 mm. Las tres filas anteriores conservan la limitación de sincronización indicada.
