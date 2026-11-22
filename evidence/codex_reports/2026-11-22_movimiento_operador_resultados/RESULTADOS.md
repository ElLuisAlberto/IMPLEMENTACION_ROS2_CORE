# Registro de movimientos ejecutados por el operador

Fecha: 22-11-2026, Asia/Manila. El operador manejó el robot; el registrador no envió órdenes de movimiento. El bag `../2026-11-22_movimiento_operador/` comprende 79,619 s, ocupa 14,5 GiB y contiene RGB (2356 mensajes), nubes de cámara (2083), tres clases (1969 de brazo, 2111 de margen y 2111 de resto), 1593 estados articulares y 1593 mensajes TF, además de diagnósticos, proximidad y órdenes JOG. La cantidad desigual de mensajes exige emparejar por marca; no permite identificar por sí sola dónde se perdieron muestras.

## Movimiento y estado

A partir de las posiciones publicadas y con desempaquetado angular, el rango observado (máximo menos mínimo) fue: J1 33,502°, J2 16,201°, J3 20,600°, J4 0,150°, J5 21,150° y J6 0,000°. Son rangos, no desplazamiento acumulado ni distancias cartesianas. Se registraron 1580 muestras de proximidad `ALLOW`, todas con geometría válida, y ninguna de las 160 muestras de diagnóstico indicó fallo. Esas muestras no garantizan que no hayan existido eventos entre ellas. El escenario PARK es una esfera virtual y no detecta obstáculos reales.

Las velocidades estimadas por diferencia finita y otros datos se encuentran en `resumen.json`; los picos son sensibles a resolución y muestreo de la posición y no son valores de velocidad física certificados. El informe no atribuye velocidades específicas a cada maniobra sin anotaciones del operador.

## Consistencia del filtro en movimiento

Se seleccionaron marcas de nubes en las que al menos una articulación presentaba una velocidad estimada mayor de 2°/s entre las muestras adyacentes del estado articular. Se encontraron **402 marcas** comunes a la nube original y las tres clases en esos intervalos; se seleccionaron **20** repartidas en el tiempo. La selección se basa en velocidad estimada, no en una medición externa ni una segmentación manual del movimiento.

En las 20 capturas analizadas, con un total de **585.617 puntos válidos**, la unión de registros de las tres clases coincidió byte por byte con la nube original tras aplicar rango y transformación experimental. La clasificación GPU coincidió con la referencia CPU para todos esos puntos. Totales en las 20 capturas: 572.312 brazo candidato, 13.288 margen incierto y 17 resto. Esas cifras son recuentos de puntos por captura sumados, no objetos ni puntos únicos de la escena.

La referencia CPU usa la misma geometría, umbrales y transformación experimental que la GPU; por tanto el resultado demuestra consistencia de implementación en las capturas analizadas, no precisión semántica frente a la realidad. No se etiquetaron de forma independiente puntos de brazo y obstáculo. No puede estimarse precisión, recall ni seguridad de preservación de objetos. La superposición sigue sin calibración extrínseca física validada.

Las tres imágenes `movimiento_inicio.png`, `movimiento_medio.png` y `movimiento_final.png` son RGB seleccionadas por cercanía entre el instante de registro de imagen y las marcas de tres nubes analizadas. Son ilustraciones aproximadas, no pares RGB–nube certificados como sincronizados ni máscaras de referencia.

`resumen.py` y `analizar_filtro.py` son scripts de análisis offline; `resumen.json` y `resultados_filtro.json` conservan sus resultados, incluyendo marcas y recuentos. El análisis reconstruye los enlaces con TF registrado y los puntos de malla y configuración conservados en `../2026-11-22_auditoria_tecnica_d435i_jaco2/`. El umbral temporal, la selección y las dependencias se pueden inspeccionar en los scripts.
