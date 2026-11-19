# Mesa del JACO2 en Gazebo y prevención

Se incorporó una mesa estática de laboratorio al mundo `jaco_empty.sdf` y se
montó el JACO2 sobre su tablero. Sus dimensiones son 1.20 m × 0.80 m, con un
tablero de 40 mm cuyo plano superior queda a 0.75 m del suelo. La base del
robot queda fijada a esa misma altura en su URDF/Xacro.

La capa preventiva conserva el obstáculo esférico existente y añade el tablero
como caja rectangular conservadora. Evalúa la distancia de cada cápsula móvil
al tablero; se excluye únicamente `base_to_shoulder`, que representa el
pedestal atornillado y contacta intencionalmente con la superficie. La distancia
se calcula con una cota inferior, por lo que no puede sobreestimar el espacio
libre entre una cápsula y la mesa.

Validación realizada: compilación de Python, expansión de Xacro con
`mount_height:=0.75`, validación XML del mundo y casos directos de contacto y
separación de la geometría caja-cápsula. No se ejecutó `pytest` porque no está
instalado en el entorno actual.
