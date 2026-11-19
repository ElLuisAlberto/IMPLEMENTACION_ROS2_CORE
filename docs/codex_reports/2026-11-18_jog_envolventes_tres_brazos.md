# Correcciones de JOG y envolventes

Se optimizó la evaluación del horizonte conservando los límites de frescura y
el comportamiento de parada. La búsqueda utiliza seis iteraciones y devuelve
una escala evaluada; se reutilizan cálculos idénticos dentro de cada ciclo.
El adaptador publica la referencia JOG enviada al controlador, y RViz muestra
esa referencia corta en lugar de una orden supervisada que pudo ser rechazada.
La referencia publicada no confirma recepción ni seguimiento físico. El
barrido se mantiene fijo tras terminar esa referencia corta; no predice órdenes
posteriores desconocidas.

Se corrigió la representación de las cápsulas actuales, incluidos extremos y
resaltado del segmento limitante. Los radios de seguridad no se redujeron ni
recalibraron. Se ocultaron por defecto las etiquetas que tapaban el modelo.

Compilación de thesis_core, thesis_simulation y thesis_multi_robot completada;
37 pruebas de geometría, política, predicción y adaptadores aprobadas.

Prueba aislada `tools/validation/probe_profile_jog.py`: CS625 registró
referencias, cuatro reducciones y paradas; UR17g registró desplazamiento y
referencias pero ninguna reducción, porque la trayectoria se alejó de la
esfera, por lo que el escenario no valida reducción; P3020 registró reducción
y paradas. El ensayo CS625 se solapó brevemente con otro proceso, y los dos
primeros ensayos usaron velocidad de obstáculo cero antes de corregirla en el
script. El criterio `pass` del script solo comprueba presencia de referencias y
reducción: no demuestra continuidad ni ausencia de colisión. El obstáculo se
mueve independientemente y puede atravesar la envolvente después de parar el
brazo. Se requiere repetir escenarios controlados por brazo, comparar
velocidad medida con escala aplicada y medir latencias TF/obstáculo, además de
verificar visualmente RViz. No se declara resuelto el comportamiento continuo.
