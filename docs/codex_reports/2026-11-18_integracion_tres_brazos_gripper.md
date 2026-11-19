# Integración simulada de tres brazos con el gripper

La célula incluye UR17g, P3020 y CS625 con el CAD completo fijo y una
envolvente de colisión conservadora incorporada a las cápsulas de seguridad.
JACO2 permanece sin esa herramienta. El montaje está estimado y no validado.

Las pruebas geométricas comprobaron que la cápsula cubre la malla STL y que
un obstáculo en la herramienta genera STOP en la clasificación offline. La
prueba independiente `validate_cell.py` se ejecutó en Gazebo para cada brazo:
controladores activos, movimiento supervisado y retorno, cancelación por
obstáculo, rechazo por STOP y datos de obstáculo caducados. Los tres pasaron.
Estas pruebas no verifican todas las configuraciones ni la autocolisión.

Se declaró la interfaz de estado `effort` y la GUI histórica muestra el valor
recibido si está disponible. Su precisión no se ha validado y no participa en
decisiones de seguridad. El gripper carece de masa e inercia en esta versión;
no se puede estudiar su carga dinámica de manera fiel. La cámara y la apertura
del gripper no se simulan. La fuente Elite sigue siendo un clon local sin
registrar como submódulo, por lo que no se reproduce automáticamente al clonar.
