# Primer incremento predictivo JACO2 simulado

La búsqueda anticipa el umbral de parada de 0,050 m con reserva experimental de 0,005 m. La reserva se limita a este escenario: los ensayos previos de parada no establecen una cota general. La escala candidata se comprueba geométricamente; si no alcanza la escala mínima, se ordena STOP. Una trayectoria no aproximante dentro de la reserva se evalúa con el margen protector habitual. Se mantienen parada y retirada dentro de la zona de STOP.

Compiló thesis_core y pasaron 41 pruebas dirigidas. En una sesión aislada con esfera fija, READY y coherencia FK/TF, hubo 104 decisiones REDUCTION; la mediana de velocidad articular durante reducción fue 0,01771 rad/s. Los 104 pares exactos de horizontes se contrajeron: mediana nominal 6,424 mm y supervisada 1,505 mm. La parada final se observó a 0,05527 m; la sonda declara el resultado global inconcluso por diseño. Traza: `2026-11-18_incremento_predictivo/traza.jsonl`.

El ensayo libre anterior observó mediana 0,06466 rad/s bajo el mismo mando, pero no se repitió en esta ejecución. No se ha demostrado desaceleración continua hasta cero, repetibilidad completa, ni alineación visual RViz. La simulación gráfica existente debe reiniciarse para cargar el código nuevo.
