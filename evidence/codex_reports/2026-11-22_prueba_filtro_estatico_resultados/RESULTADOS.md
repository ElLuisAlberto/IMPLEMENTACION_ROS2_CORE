# Prueba observacional del filtro con brazo inmóvil

Fecha: 22-11-2026, hora local Asia/Manila. Se realizó una grabación pasiva sin comandos de movimiento y sin cambiar el filtro. Objetivo: verificar partición, equivalencia de clasificación frente a una referencia CPU y estabilidad temporal en la escena disponible. Estos ensayos no establecen sensibilidad ni especificidad frente a etiquetas físicas del brazo y objetos.

## Material y método

- Bag: `../2026-11-22_prueba_filtro_estatico/`, duración 15,703 s y tamaño 2,8 GiB. Incluye RGB, nube original, tres salidas, TF y estados articulares; sus metadatos conservan recuentos y tiempos.
- Se identificaron 312 marcas presentes en la nube original y en las tres clases. Se seleccionaron 12 repartidas a lo largo de esas marcas; selección determinista mediante índices equiespaciados, no aleatoria.
- Para cada una se aplicó a la nube de entrada el mismo rango radial de 0,10–1,50 m y la matriz experimental del programa C++ en aritmética de doble precisión con salida float32. Se comparó la unión multiconjunto de registros completos de salida con la entrada transformada, incluidos los campos adicionales como color.
- Se reconstruyeron las mallas de los 13 enlaces a la marca de la nube usando el TF del bag y los puntos de malla preservados en la auditoría técnica. Una implementación CPU con cKDTree calculó distancias a malla y articulaciones y evaluó las seis cápsulas; se compararon sus tres etiquetas con las salidas de GPU. Es una referencia algorítmica basada en el mismo modelo, no una verdad de terreno independiente.
- Para pose se revisaron los valores publicados de las seis articulaciones en los 294 mensajes registrados. Para frecuencia se usaron tiempos de recepción en el bag; la grabación y el transporte pueden introducir pérdidas y carga.
- Script, resultados detallados, marcas de tiempo y conteos: `analizar.py` y `resultados.json`. Entrada, matriz y archivo de cápsulas se fijaron desde la copia preservada en `../2026-11-22_auditoria_tecnica_d435i_jaco2/`. Ejecutar desde la raíz del workspace con ROS Humble, NumPy, SciPy y PyYAML disponibles.

## Resultados

| Ensayo | Resultado observado | Interpretación |
|---|---|---|
| Postura publicada | 294 muestras; rango de cada una de las seis articulaciones: 0 rad | La señal publicada fue constante; no equivale a una medición externa de inmovilidad. |
| Integridad de partición | 12 de 12 capturas; multiconjunto idéntico byte a byte a la entrada válida transformada | En esas capturas no se perdió ni duplicó ningún registro al formar el conjunto de salidas. No demuestra semántica física correcta. |
| Referencia CPU frente a GPU | 0 diferencias de etiqueta en 12 capturas, 364.980 puntos comparados | Los dos cálculos coincidieron para las muestras y el modelo utilizados. |
| Frecuencia observada de resto | 391 mensajes; 26,51 Hz, mediana entre recepciones 32,15 ms, máximo 131,12 ms | Resultado durante grabación; no acredita frecuencia sostenida ni latencia máxima del sistema. |
| Tamaño de clases, todas las muestras recibidas | Brazo: mediana 29.552 puntos; incierto: mediana 809; resto: mediana 0, máximo 53 | La escena estaba mayoritariamente vacía dentro del rango visible fuera del brazo; esta no es una prueba positiva con obstáculo. |

En el bag, las tres salidas se grabaron en cantidades diferentes: 353 de brazo, 391 de incertidumbre y 391 de resto, mientras que se registraron 407 nubes de entrada. Se compararon solo las 312 marcas comunes; por QoS, suscripciones y grabación, los conteos no permiten determinar por sí solos dónde se perdieron mensajes. Las doce capturas elegidas tenían resto vacío; por ello la prueba de clasificación de esas capturas **no ejercita puntos positivos de la clase resto**.

## Limitaciones y próximos experimentos

La matriz de superposición es experimental y la clasificación CPU usa las mismas mallas y umbrales que la GPU. Ambas pueden coincidir y estar equivocadas frente al robot o un obstáculo real. La escena no ofrece una máscara manual de referencia ni un objeto visible conocido dentro del espacio observado, por lo que no se puede calcular precisión, recall, falsos positivos o falsos negativos físicos. El brazo inmóvil no pone a prueba sincronización bajo movimiento ni oclusiones cambiantes.

Como pruebas posteriores con brazo inmóvil: capturar un objeto de referencia con dimensiones y posición conocidas en varias zonas visibles, incluida una distancia próxima al brazo sin contacto; conservar referencia de imagen y profundidad, etiquetar brazo y objeto independientemente y cuantificar los puntos del objeto conservados y los residuos del brazo. Cualquier intervención física en el área debe ajustarse al procedimiento del laboratorio. Comparar varias capturas y condiciones, registrar resultados negativos y la incertidumbre de calibración. Mantener la salida como observación experimental hasta validación.
