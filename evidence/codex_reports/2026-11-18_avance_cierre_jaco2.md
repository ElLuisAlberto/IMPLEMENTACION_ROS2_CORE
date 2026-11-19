# Avance del cierre JACO2 simulado

Estado: **incompleto**. La sonda registra mensajes completos y tiempos monotónicos, y permite ensayo libre y liberación del JOG. Las trazas están en `2026-11-18_jaco_probe_trazas/`. La preparación aún no verifica controlador, reloj y TF completos; por ello no son una referencia aceptada.

La pasada libre preliminar registró 449 decisiones y referencias, escala 1, desplazamiento de 1,092 rad y velocidad máxima de 0,08694 rad/s. La aproximación con esfera fija reprodujo STOP a 0,049951 m sin REDUCTION. Tras STOP se observó velocidad de 0,06882 rad/s a 13 ms y 0,000856 rad/s a 143 ms; estas muestras no constituyen un presupuesto de frenado validado. La liberación del JOG se ensayó por separado, pero no equivale a STOP geométrico.

Se añadió caducidad a los marcadores de cápsulas y limpieza ante pausa o retroceso del reloj. Cuatro pruebas dirigidas pasaron y `thesis_core` compiló. La integración de TF y reloj y la invalidación de la predicción estructurada siguen pendientes.

No se modificó la política preventiva: falta caracterizar formalmente latencia, reposo y distancia de frenado, y comparar cinemática y TF por cápsula antes de fijar una reserva. También quedan pendientes la matriz completa, tres sesiones independientes y la comprobación con interfaz, Gazebo y RViz.
