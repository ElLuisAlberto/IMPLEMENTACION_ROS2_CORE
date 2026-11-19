# Visualización de seguridad y ensayo JACO2

Se actualizó RViz para dibujar una sola envolvente actual y usar un color fijo
en el volumen futuro. La predicción de ejecución sin decisión correlacionada y
reciente queda indisponible. La referencia corta JOG se identifica por su duración.
La interfaz distingue proximidad, decisión, referencia enviada y velocidad medida,
y comprueba frescura de mensajes y avance del reloj simulado.

Se compilaron thesis_core, thesis_description y thesis_ui. Las 51 pruebas
dirigidas de horizonte e interfaz pasaron, incluida la recuperación del reloj.

El ensayo aislado de JACO2 con obstáculo fijo registró movimiento y STOP, pero
ninguna decisión REDUCTION. El registro quedó en /tmp/jog_probe_bjsle_r5/launch.log.
El STOP de distancia actual se activó cerca de 0,050 m mientras la distancia
nominal prevista era aproximadamente 0,046 m y el margen predictivo 0,020 m.
Debe investigarse la relación entre umbrales y predicción con pruebas que
midan velocidad y continuidad antes de afirmar que la reducción está validada.
Los tres perfiles comerciales todavía requieren la matriz de ensayos en vivo.
